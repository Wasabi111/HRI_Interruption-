#!/usr/bin/env python

# Copyright 2017 Google Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Google Cloud Speech API sample application using the streaming API.
NOTE: This module requires the additional dependency `pyaudio`. To install
using pip:
    pip install pyaudio
Example usage:
    python transcribe_streaming_mic.py
"""

# [START speech_transcribe_streaming_mic]
from __future__ import division

import re
import sys

import rospy
from std_msgs.msg import Int64
import rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('robot_interruption_detection')
import os
os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = package_path + '/GCloud_SpeechAPI_Cred.json'

from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
import pyaudio
from six.moves import queue

# Audio recording parameters
RATE = 44100
CHUNK = int(RATE / 10)  # 100ms

staring_status = [0, 0]
staring_start = -1
streaming_start = -1
pub = rospy.Publisher('/hiro/language/easement', Int64, queue_size=1)

class MicrophoneStream(object):
    """Opens a recording stream as a generator yielding the audio chunks."""
    def __init__(self, rate, chunk):
        self._rate = rate
        self._chunk = chunk

        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()
        self.closed = True

    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            # The API currently only supports 1-channel (mono) audio
            # https://goo.gl/z757pE
            channels=1, rate=self._rate,
            input=True, frames_per_buffer=self._chunk,
            # Run the audio stream asynchronously to fill the buffer object.
            # This is necessary so that the input device's buffer doesn't
            # overflow while the calling thread makes network requests, etc.
            stream_callback=self._fill_buffer,
        )

        self.closed = False

        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        """Continuously collect data from the audio stream, into the buffer."""
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def generator(self):
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b''.join(data)


def listen_print_loop(responses):
    """Iterates through server responses and prints them.
    The responses passed is a generator that will block until a response
    is provided by the server.
    Each response may contain multiple results, and each result may contain
    multiple alternatives; for details, see https://goo.gl/tjCPAU.  Here we
    print only the transcription for the top alternative of the top result.
    In this case, responses are provided for interim results as well. If the
    response is an interim one, print a line feed at the end of it, to allow
    the next result to overwrite it, until the response is a final one. For the
    final one, print a newline to preserve the finalized transcription.
    """
    global staring_status
    global staring_start
    global streaming_start
    num_chars_printed = 0
    for response in responses:
        if not response.results:
            continue

        # The `results` list is consecutive. For streaming, we only care about
        # the first result being considered, since once it's `is_final`, it
        # moves on to considering the next utterance.
        result = response.results[0]
        if not result.alternatives:
            continue

        # Display the transcription of the top alternative.
        transcript = result.alternatives[0].transcript

        # Display interim results, but with a carriage return at the end of the
        # line, so subsequent lines will overwrite them.
        #
        # If the previous result was longer than this one, we need to print
        # some extra spaces to overwrite the previous result
        overwrite_chars = ' ' * (num_chars_printed - len(transcript))

        if not result.is_final:
            sys.stdout.write(transcript + overwrite_chars + '\r')
            sys.stdout.flush()

            num_chars_printed = len(transcript)

        else:
            print(transcript + overwrite_chars)

            # Exit recognition if any of the transcribed phrases could be
            # one of our keywords.
            global pub
            if re.search(r'\b(exit|quit|stop)\b', transcript, re.I):
                pub.publish(0)
            elif re.search(r'\b(go|ahead|continue)\b', transcript, re.I):
                pub.publish(1)

            num_chars_printed = 0

        if (rospy.get_time() - streaming_start >= 30):
            streaming_start = -1
            print('Exiting..')
            break

def is_staring():
    global staring_status
    if (staring_status[0] == 1 or staring_status[1] == 1):
        return True
    return False

def screen_callback(data):
    global staring_status
    global staring_start
    global streaming_start
    staring_status[0] = data.data
    if (is_staring() and staring_start == -1):
        staring_start = rospy.get_time()
    elif (is_staring() and staring_start > 0):
        if (rospy.get_time() - staring_start >= 0.2 and streaming_start == -1):
            streaming_start = rospy.get_time()
            start_streaming()
    elif (not is_staring()):
        staring_start = -1

def gripper_callback(data):
    global staring_status
    global staring_start
    global streaming_start
    staring_status[1] = data.data
    if (is_staring() and staring_start == -1):
        staring_start = rospy.get_time()
    elif (is_staring() and staring_start > 0):
        if (rospy.get_time() - staring_start >= 0.2 and streaming_start == -1):
            streaming_start = rospy.get_time()
            start_streaming()
    elif (not is_staring()):
        staring_start = -1

def start_streaming():
    language_code = 'en-US'  # a BCP-47 language tag

    client = speech.SpeechClient()
    config = types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code=language_code)
    streaming_config = types.StreamingRecognitionConfig(
        config=config,
        interim_results=True)

    with MicrophoneStream(RATE, CHUNK) as stream:
        audio_generator = stream.generator()
        requests = (types.StreamingRecognizeRequest(audio_content=content)
                    for content in audio_generator)

        responses = client.streaming_recognize(streaming_config, requests)

        # Now, put the transcription responses to use.
        listen_print_loop(responses)

def main():
    # See http://g.co/cloud/speech/docs/languages
    # for a list of supported languages.
    rospy.init_node('audio_stream_recognition', anonymous=True)
    rospy.Subscriber('/hiro/lookat_screen', Int64, screen_callback)
    rospy.Subscriber('/hiro/lookat_gripper', Int64, gripper_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
# [END speech_transcribe_streaming_mic]