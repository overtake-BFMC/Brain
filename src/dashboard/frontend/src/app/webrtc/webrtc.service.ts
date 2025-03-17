import { Injectable } from '@angular/core';
import { Subscription, interval } from 'rxjs';
import { takeWhile } from 'rxjs/operators';
import { WebSocketService } from '../webSocket/web-socket.service';

@Injectable({
  providedIn: 'root',
})
export class WebRTCService {
  private peerConnection!: RTCPeerConnection;
  private videoElement!: HTMLVideoElement;
  private WebRTCAnswerSubscription: Subscription | undefined;
  private IceCandidateSubscription: Subscription | undefined;
  private offerSubscription: Subscription | null = null;
  private isConnected = false;

  constructor(private webSocketService: WebSocketService) {
    // this.webSocketService.receiveUnhandledEvents().subscribe(({ channel, data }) => {
    //   if (channel === 'WebRTCAnswer') {
    //     this.handleAnswer(data.value);
    //   } else if (channel === 'ice_candidate') {
    //     this.handleIceCandidate(data);
    //   }
    // });

    this.WebRTCAnswerSubscription = this.webSocketService.receiveWebRTCAnswer().subscribe(
      (message) => {
        this.handleAnswer(message.value)
      },
      (error) => {
        console.error('Error receiving RTC data: ', error)
      }
    );

    this.IceCandidateSubscription = this.webSocketService.receiveICECandidate().subscribe(
      (message) => {
        this.handleIceCandidate(message)
      },
      (error) => {
        console.error('Error getting ICE Candidate: ', error)
      }
    );

    this.peerConnection = new RTCPeerConnection({
        iceServers: []
      });

    this.peerConnection.onicecandidate = (event) => {
      if (event.candidate) {
        console.log('Generated ICE candidate:', event.candidate);
        //this.webSocketService.sendIceCandidateToFlask({ candidate: event.candidate });
        //this.webSocketService.sendMessageToFlask(`{"Name": "ICECandidate", "Value": "${event.candidate}"}`);
      }
    };
  
      // Log ICE candidate errors
    this.peerConnection.onicecandidateerror = (event) => {
      console.error('ICE candidate error:', event);
    };
  
      // Log connection state changes
    this.peerConnection.onconnectionstatechange = () => {
      console.log('Peer Connection state:', this.peerConnection.connectionState);
      if(this.peerConnection.connectionState == 'connected') {
        this.onPeerConnected();
      } 
    };
  
      // Log signaling state changes
    this.peerConnection.onsignalingstatechange = () => {
      console.log('Signaling state:', this.peerConnection.signalingState);
    };

    this.peerConnection.oniceconnectionstatechange = () => {
      console.log("ICE Connection State: ", this.peerConnection.iceConnectionState);
    };

    this.peerConnection.onicegatheringstatechange = () => {
      if (this.peerConnection.iceGatheringState === 'complete') {
        console.log("ICE gathering complete.");
        console.log("FINAL SDP OFFER: ", this.peerConnection.localDescription)
      }
    };

    this.peerConnection.ontrack = (event) => {
      if (event.streams.length > 0) {
        console.log('Remote stream received:', event.streams[0]);
        if (this.videoElement) {
          this.videoElement.srcObject = event.streams[0];
        }
      }
    };
  }

  async handleAnswer(data: RTCSessionDescriptionInit) {
    console.log('Answer Data:', data);
    await this.peerConnection.setRemoteDescription(new RTCSessionDescription(data));
    const sdpString = data.sdp ? data.sdp.toString() : "";
    const iceCandidate = this.extractCandidatesFromSDP(sdpString );
    for(const candidate of await iceCandidate) {
      console.log("ICE Candidate Recieved: ", candidate);
      //await this.peerConnection.addIceCandidate(candidate);
    }
  }

  async extractCandidatesFromSDP(sdp: string ): Promise<RTCIceCandidate[]> {
    const candidates: RTCIceCandidate[] = [];
    const lines = sdp?.split("\n");
    
    for(const line of lines) {
      if(line.startsWith('a=candidate:')) {
        try {
          const candidate = new RTCIceCandidate({ candidate: line.trim(), sdpMid: "0"})
          candidates.push(candidate)
        } catch (error) {
          console.error("Error parsing ICE candidate: ", error)
        }
      }
    }

    return candidates;
  }

  async handleIceCandidate(data: any) {
    this.peerConnection.addIceCandidate(new RTCIceCandidate(data.candidate));
    console.log('Ice Candidate Data: ', data)
  }

  public async startStreaming(videoElement: HTMLVideoElement): Promise<void> {
    this.videoElement = videoElement;

    this.peerConnection.addTransceiver('video', { direction: 'recvonly' });
    this.peerConnection.addTransceiver('audio', { direction: 'recvonly' });

    this.startOfferLoop();

    //const offer = await this.peerConnection.createOffer();
    //await this.peerConnection.setLocalDescription(offer);
    
    //this.webSocketService.sendOfferToFlask({sdp: this.peerConnection.localDescription?.sdp , type: this.peerConnection.localDescription?.type});
    /*
    this.peerConnection.ontrack = (event) => {
      if (event.streams.length > 0) {
        console.log("Remote stream received:", event.streams[0]);
        this.videoElement.srcObject = event.streams[0];
      }
    };
    */
  }

  private startOfferLoop(): void {
    if (this.offerSubscription) {
      this.offerSubscription.unsubscribe();
    }

    this.offerSubscription = interval(2000)
      .pipe(takeWhile(() => !this.isConnected))
      .subscribe(() => this.sendOffer());
  }

  private async sendOffer(): Promise<void> {
    try {
      const offer = await this.peerConnection.createOffer();
      await this.peerConnection.setLocalDescription(offer);
      console.log('Sending SDP Offer!');

      this.webSocketService.sendOfferToFlask({
        sdp: this.peerConnection.localDescription?.sdp,
        type: this.peerConnection.localDescription?.type,
      });
    } catch (error) {
      console.error('Failed to create or send offer: ', error);
    }
  }

  private stopOfferLoop(): void {
    this.offerSubscription?.unsubscribe();
    this.offerSubscription = null;
  }

  private onPeerConnected(): void {
    this.isConnected = true;
    this.stopOfferLoop();
    console.log('Peer successfully connected. Stopping SDP offers.')
  }
}
