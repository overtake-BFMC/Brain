import { Injectable } from '@angular/core';
import { WebSocketService } from '../webSocket/web-socket.service';
import { SpeedometerComponent } from '../cluster/speedometer/speedometer.component';

@Injectable({
  providedIn: 'root',
})
export class WebRTCService {
  private peerConnection!: RTCPeerConnection;
  private videoElement!: HTMLVideoElement;

  constructor(private webSocketService: WebSocketService) {
    this.webSocketService.receiveUnhandledEvents().subscribe(({ channel, data }) => {
      if (channel === 'WebRTCAnswer') {
        this.handleAnswer(data.value);
      } else if (channel === 'ice_candidate') {
        this.handleIceCandidate(data);
      }
    });

    this.peerConnection = new RTCPeerConnection({
        iceServers: []
      });

      this.peerConnection.onicecandidate = (event) => {
        if (event.candidate) {
          console.log('Generated ICE candidate:', event.candidate);
          this.webSocketService.sendIceCandidateToFlask({ candidate: event.candidate });
          //this.webSocketService.sendMessageToFlask(`{"Name": "ICECandidate", "Value": "${event.candidate}"}`);
        } else {
          console.log('ICE gathering complete');
        }
      };
  
      // Log ICE candidate errors
      this.peerConnection.onicecandidateerror = (event) => {
        console.error('ICE candidate error:', event);
      };
  
      // Log connection state changes
      this.peerConnection.onconnectionstatechange = () => {
        console.log('Peer Connection state:', this.peerConnection.connectionState);
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
  }

  async handleAnswer(data: RTCSessionDescriptionInit) {
    console.log('Answer Data:', data);
    await this.peerConnection.setRemoteDescription(new RTCSessionDescription(data));
    const sdpString = data.sdp ? data.sdp.toString() : "";
    const iceCandidate = this.extractCandidatesFromSDP(sdpString );
    for(const candidate of await iceCandidate) {
      console.log("ICE Candidate Recieved: ", candidate);
      await this.peerConnection.addIceCandidate(candidate);
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

    const offer = await this.peerConnection.createOffer();
    await this.peerConnection.setLocalDescription(offer);

    this.webSocketService.sendOfferToFlask({sdp: offer.sdp , type: offer.type});
    //this.webSocketService.sendMessageToFlask(`{"Name": "WebRTCOffer", "Value": "'${offer.sdp}'"}`);

    this.peerConnection.ontrack = (event) => {
      if (event.streams.length > 0) {
        console.log("Remote stream received:", event.streams[0]);
        this.videoElement.srcObject = event.streams[0];
      }
    };
  }
}
