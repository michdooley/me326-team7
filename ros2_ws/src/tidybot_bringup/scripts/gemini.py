import os
import google.generativeai as genai

model = genai.GenerativeModel("gemini-2.0-flash")
response = model.generate_content("Give me a random fact.")
print(response.text)
