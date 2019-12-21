#!/usr/bin/env python
#-*- coding: utf-8 -*-

import re
import sys

from SpeechRecog_stream import ResumableMicrophoneStream

from google.cloud import speech

import roslib
import rospy
from gcp_speech_recognition.srv import SpeechRecog, SpeechRecogResponse

SAMPLE_RATE = 16000
CHUNK_SIZE = int(SAMPLE_RATE / 10)  # 100ms

def listen_print_loop(responses, stream):
    responses = (r for r in responses if (
            r.results and r.results[0].alternatives))

    num_chars_printed = 0

    for response in responses:
        if not response.results:
            continue

        result = response.results[0]

        if not result.alternatives:
            continue

        top_alternative = result.alternatives[0]
        transcript = top_alternative.transcript

        overwrite_chars = ' ' * (num_chars_printed - len(transcript))

        if not result.is_final:
            sys.stdout.write(transcript + overwrite_chars + '\r')
            sys.stdout.flush()

            num_chars_printed = len(transcript)

        else:
            print(transcript + overwrite_chars)
            return transcript


            if re.search(r'\b(exit|quit|goodbye)\b', transcript, re.I):
                print('Exiting..')
                stream.closed = True
                break

            num_chars_printed = 0


def google_speech_api(_dammy):
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


            return listen_print_loop(responses, stream)

def stt_server():
    rospy.init_node('gcp_speech_recognition')
    srv = rospy.Service('/speech_recog', SpeechRecog, google_speech_api)
    rospy.loginfo("Ready to gcp_speech_recognition srvserver")
    rospy.spin()

if __name__ == '__main__':
    stt_server()
