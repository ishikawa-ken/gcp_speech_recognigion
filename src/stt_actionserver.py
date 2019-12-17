#!/usr/bin/env python
#-*- coding: utf-8 -*-

# [START transcribe]
import re
import sys

import roslib
import rospy
import actionlib
from gcp_speech_recognition.msg import *

from SpeechRecog_stream import ResumableMicrophoneStream

from google.cloud import speech

SAMPLE_RATE = 16000
CHUNK_SIZE = int(SAMPLE_RATE / 10)  # 100ms

#<<---------------------
# #goal definition
# ---
# #result definition
# string recog_result
# ---
# #feedback
# string recog_result
#--------------------->>

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
