import sounddevice as sd
import wave
import os
import google.generativeai as genai



from google.cloud import speech_v1p1beta1 as speech
def transcribe_audio(filename, sample_rate=16000):
    """
    Uses Google Cloud Speech-to-Text to transcribe the given audio file and returns the transcription as a string.
    """
    client = speech.SpeechClient()

    with open(filename, 'rb') as audio_file:
        content = audio_file.read()

    audio = speech.RecognitionAudio(content=content)
    config = speech.RecognitionConfig(
        sample_rate_hertz=sample_rate,
        language_code='en-US',
        enable_automatic_punctuation=True,
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
    )

    response = client.recognize(config=config, audio=audio)

    full_transcript = []
    for result in response.results:
        full_transcript.append(result.alternatives[0].transcript)

    return " ".join(full_transcript)

def record_audio(filename, duration=5, sample_rate=16000):
    """
    Records audio from the microphone for a given duration and saves it as a WAV file.
    """
    print("Recording started...")
    recorded_data = sd.rec(
        int(duration * sample_rate),
        samplerate=sample_rate,
        channels=1,
        dtype='int16'
    )
    sd.wait()
    print("Recording done. Saving audio...")

    with wave.open(filename, 'wb') as wf:
        wf.setnchannels(1)                 # Mono audio
        wf.setsampwidth(2)          
        wf.setframerate(sample_rate)
        wf.writeframes(recorded_data.tobytes())
    print(f"Audio saved to {filename}")


def main():
    filename = "test.wav"
    record_audio(filename, duration=5, sample_rate=16000)

    transcription = transcribe_audio(filename)
    print("Transcription:\n", transcription)
    
    model = genai.GenerativeModel("gemini-2.0-flash")
    message = f"return the verb and the object, based on what the next sentense." + transcription
    response = model.generate_content(message)
    print(response.text)


if __name__ == "__main__":
    main()