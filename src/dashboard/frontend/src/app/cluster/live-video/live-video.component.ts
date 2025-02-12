import { CommonModule } from '@angular/common';
import { Component, AfterViewInit, ViewChild, ElementRef } from '@angular/core';
import { WebRTCService } from '../../webrtc/webrtc.service';

@Component({
  selector: 'app-live-video',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './live-video.component.html',
  styleUrls: ['./live-video.component.css']
})
export class LiveVideoComponent implements AfterViewInit {
    public loading: boolean = false;
  @ViewChild('videoElement', { static: false }) videoElement!: ElementRef<HTMLVideoElement>;

  constructor(private webRTCService: WebRTCService) {}

  ngAfterViewInit(): void {
    if (this.videoElement) {
      this.webRTCService.startStreaming(this.videoElement.nativeElement);

    }
  }
}
