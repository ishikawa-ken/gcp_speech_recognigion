#!/usr/bin/env python
#-*- coding: utf-8 -*-

# [START transcribe]
import time
import re
import sys

import roslib
import rospy
import actionlib
from gcp_speech_recognition.msg import *
#<<---------------------
# #goal definition
# ---
# #result definition
# string recog_result
# ---
# #feedback
# string recog_result
#--------------------->>

from google.cloud import speech

import pyaudio
from six.moves import queue

# Audio recording parameters
STREAMING_LIMIT = 290000
SAMPLE_RATE = 16000
CHUNK_SIZE = int(SAMPLE_RATE / 10)  # 100ms


def get_current_time():
    return int(round(time.time() * 1000))


def duration_to_secs(duration):
    return duration.seconds + (duration.nanos / float(1e9))

class ResumableMicrophoneStream:
    def __init__(self, rate, chunk_size):
        self._rate = rate
        self._chunk_size = chunk_size
        self._num_channels = 1
        self._max_replay_secs = 5
        self._buff = queue.Queue()
        self.closed = True
        self.start_time = get_current_time()
        self._bytes_per_sample = 2 * self._num_channels
        self._bytes_per_second = self._rate * self._bytes_per_sample
        self._bytes_per_chunk = (self._chunk_size * self._bytes_per_sample)
        self._chunks_per_second = (
                self._bytes_per_second // self._bytes_per_chunk)

    def __enter__(self):
        self.closed = False
        self._audio_interface = pyaudio.PyAudio()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            channels=self._num_channels,
            rate=self._rate,
            input=True,
            frames_per_buffer=self._chunk_size,
            stream_callback=self._fill_buffer,
        )
        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(self, in_data, *args, **kwargs):
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def generator(self):
        while not self.closed:
            if get_current_time() - self.start_time > STREAMING_LIMIT:
                self.start_time = get_current_time()
                break

            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b''.join(data)


class CallApi(object):

    _feedback = gcp_speech_recognition.msg.SpeechRecogFeedback()
    _result   = gcp_speech_recognition.msg.SpeechRecogResult()

    def __init__(self):
        rospy.init_node('gcp_speech_recognition')
        self._action_name = rospy.get_name()
        self._as = actionlib.SimpleActionServer(name=self._action_name,\
                                                ActionSpec=SpeechRecogAction,\
                                                auto_start=False)
        self._as.start()
        rospy.loginfo("Ready to gcp_speech_recognition server.")


    def listen_print_loop(self, responses, stream):
        responses = (r for r in responses if (
                r.results and r.results[0].alternatives))

        num_chars_printed = 0
        for response in responses:
            #print(response)

            if not response.results:
                continue

            result = response.results[0]
            if not result.alternatives:
                continue

            top_alternative = result.alternatives[0]
            transcript = top_alternative.transcript

            overwrite_chars = ' ' * (num_chars_printed - len(transcript))

            if not result.is_final:
                self._feedback = transcript + overwrite_chars
                rospy.loginfo('%s: Preempt' % self._action_name)
                self._as.set_preempted()
                self._as.publish_feedback(self._feedback)
                sys.stdout.write(transcript + overwrite_chars + '\r')
                sys.stdout.flush()

                num_chars_printed = len(transcript)

            else:
                print(transcript + overwrite_chars)
                self._result = transcript + overwrite_chars
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._as.set_succeeded(self._result)
                break

                if re.search(r'\b(exit|quit|goodbye)\b', transcript, re.I):
                    print('Exiting..')
                    break

                num_chars_printed = 0


    def google_speech_api(self):
        client = speech.SpeechClient()
        config = speech.types.RecognitionConfig(
            encoding=speech.enums.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=SAMPLE_RATE,
            language_code='en-US',
            max_alternatives=1,
            enable_word_time_offsets=True)
        streaming_config = speech.types.StreamingRecognitionConfig(
            config=config,
            interim_results=True)

        mic_manager = ResumableMicrophoneStream(SAMPLE_RATE, CHUNK_SIZE)

        print('Say "Quit" or "Exit" to terminate the program.')

        with mic_manager as stream:
            while not stream.closed:
                audio_generator = stream.generator()
                requests = (speech.types.StreamingRecognizeRequest(
                    audio_content=content)
                    for content in audio_generator)

                responses = client.streaming_recognize(streaming_config,
                                                       requests)
                self.listen_print_loop(responses, stream)


if __name__ == '__main__':
    CallApi().google_speech_api()
    rospy.spin()

# [END transcribe]
