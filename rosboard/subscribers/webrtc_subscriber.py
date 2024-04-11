import asyncio
import base64
import json
import cv2
import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription

# Use this to receive videos! (when i get it to start working)

class WebRTCSubscriber:
    def __init__(self, offer, callback):
        self.callback = callback
        self.pc = RTCPeerConnection()
        self.pc.ontrack = self.on_track
        # self.offer = offer
        asyncio.ensure_future(self.start(offer))

    async def start(self, offer):
        await self.pc.setRemoteDescription(RTCSessionDescription(
            sdp=offer["sdp"], 
            type=offer["type"]))
        answer = await self.pc.createAnswer()
        await self.pc.setLocalDescription(answer)

        return {"sdp": self.pc.localDescription.sdp, 
                "type": self.pc.localDescription.type}
    
    def on_track(self, track):
        @track.on("data")
        async def on_data(data):
            if self.callback is not None:
                self.callback(data)
                
