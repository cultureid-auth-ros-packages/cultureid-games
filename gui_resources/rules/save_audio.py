# coding=utf-8
from __future__ import division
from subprocess import Popen
import os
import time
import re
import sys
import wave
import io
import pyaudio
from six.moves import queue
import numpy as np
from google.oauth2 import service_account
from google.cloud import speech_v1 as speech
from google.cloud.speech import enums
from google.cloud import texttospeech_v1 as texttospeech

os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = 'indytherobot-9e663be7d21d.json'



#-------------------------------------------------------------------------------
def text_to_speech(text,output_file):

    SCOPES = ['https://www.googleapis.com/auth/cloud-platform']
    cred = service_account.Credentials.from_service_account_file('indytherobot-9e663be7d21d.json', scopes=SCOPES)

    client=texttospeech.TextToSpeechClient(credentials=cred)


    #output of: print(client.list_voices())
    #voices {
      #language_codes: "el-GR"
      #name: "el-GR-Wavenet-A"
      #ssml_gender: FEMALE
      #natural_sample_rate_hertz: 24000
    #}
    #voices {
      #language_codes: "el-GR"
      #name: "el-GR-Standard-A"
      #ssml_gender: FEMALE
      #natural_sample_rate_hertz: 24000
    #}

    input=texttospeech.types.SynthesisInput(text=text)

    voice = texttospeech.types.VoiceSelectionParams(
        language_code='el-GR',
        ssml_gender=texttospeech.enums.SsmlVoiceGender.FEMALE,
        name='el-GR-Wavenet-A' # Better than el-GR-Standard-A
    )


    audio_config=texttospeech.types.AudioConfig(
        audio_encoding=texttospeech.enums.AudioEncoding.LINEAR16,
        speaking_rate=0.9,
        #pitch=2,
        #effects_profile_id=[effects_profile_id]

    )

    # https://stackoverflow.com/questions/55291174/error-in-python-cryptography-module-rsaprivatekey-object-has-no-attribute-si
    response=client.synthesize_speech(
        input_=input,
        voice=voice,
        audio_config=audio_config
    )


    speech_duration = 0.0
    with wave.open(io.BytesIO(response.audio_content), 'rb') as f:
        width = f.getsampwidth()
        channels = f.getnchannels()
        rate = f.getframerate()
        frames = f.getnframes()
        speech_duration = frames / float(rate)

    pa = pyaudio.PyAudio()

    pa_stream = pa.open(
        format=pyaudio.get_format_from_width(width),
        channels=channels,
        rate=rate,
        output=True)

  # The response's audio_content is binary.
    with open(output_file, "wb") as out:
        out.write(response.audio_content)
        print('Audio content written to file')








if __name__ == "__main__":

    text = 'Γειά σας. Είμαι ο indy, το ρομπότ του αρχαιολογικού μουσείου θεσσαλονίκης και μαζί μου θα μάθετε για την ζωή στην Αρχαία Μακεδονία παίζοντας μερικά παιχνίδια.'
    output_file="rules00.mp3"
    text_to_speech(text,output_file)

    text = 'Αρχικά θα χωριστείτε σε τέσσερις ομάδες.'
    output_file="rules01.mp3"
    text_to_speech(text,output_file)

    text = 'Κάθε ομάδα ξεκινάει με δέκα πόντους.'
    output_file="rules02.mp3"
    text_to_speech(text,output_file)

    text = 'Για κάθε σωστή απάντηση παίρνετε τέσσερις πόντους.'
    output_file="rules03.mp3"
    text_to_speech(text,output_file)

    text = 'Για κάθε λάθος απάντηση χάνετε έναν πόντο, γι’ αυτό να είστε πολύ προσεκτικοί με τις απαντήσεις σας!'
    output_file="rules04.mp3"
    text_to_speech(text,output_file)

    text = 'Οι πόντοι της κάθε ομάδας θα φαίνονται επίσης πάνω στα κουμπιά τα οποία μπορείτε να χρησιμοποιείτε για να αλλάζετε την ομάδα που θέλει να παίξει.'
    output_file="rules05.mp3"
    text_to_speech(text,output_file)

    text = 'Αν μια ομάδα θέλει να τερματίσει το παιχνίδι της τοτε μπορεί να πατήσει στο κουμπί της ομάδας της και να επιλέξει τερματισμό.'
    output_file="rules06.mp3"
    text_to_speech(text,output_file)

    text = 'Κάποιες ερωτήσεις έχουν μοναδική απάντηση. Για να απαντήσετε πιέστε πάνω στην απάντηση που έχετε αποφασίσει στην οθόνη μου.'
    output_file="rules07.mp3"
    text_to_speech(text,output_file)

    text = 'Άλλες ερωτήσεις έχουν παραπάνω από μία σωστή απάντηση. Μόνο αν διαλέξετε όλες τις σωστές απαντήσεις θα έχετε απαντήσει σωστά και θα σάς δώσω τους τέσσερις πόντους.'
    output_file="rules08.mp3"
    text_to_speech(text,output_file)

    text = 'Σε αυτήν την περίπτωση πρέπει να επιλέξετε όλες τις απαντήσεις που θεωρείτε σωστές και μετά να πιέσετε το κουμπί της ερώτησης.'
    output_file="rules09.mp3"
    text_to_speech(text,output_file)

    text = 'Σε κάποιες ερωτήσεις μπορεί να χρειαστεί να βρείτε μία κάρτα μέσα στο Μουσείο. Τότε θα πρέπει να την φέρετε σε μένα και να την ακουμπήσετε στο σημείο που σας δείχνω.'
    output_file="rules10.mp3"
    text_to_speech(text,output_file)

    text = 'Kρατηστε την εκεί και πατήστε το κουμπί της ερώτησης για να σας απαντήσω αν φέρατε τη σωστή κάρτα.'
    output_file="rules11.mp3"
    text_to_speech(text,output_file)

    text = 'Όλες οι απαντήσεις κρύβονται στο χώρο γύρω σας.'
    output_file="rules12.mp3"
    text_to_speech(text,output_file)

    text = 'Μπορεί να βρίσκονται μέσα σε μία προθήκη.'
    output_file="rules13.mp3"
    text_to_speech(text,output_file)

    text = 'ή μπορεί να είναι κάπου γραμμένες γύρω από τα εκθέματα.'
    output_file="rules14.mp3"
    text_to_speech(text,output_file)

    text = 'Δεν χρειάζεται να μαντέψετε. Αρκεί να είστε προσεκτικοί σαν εξερευνητές. Καλή επιτυχία και καλή διασκέδαση!'
    output_file="rules15.mp3"
    text_to_speech(text,output_file)
