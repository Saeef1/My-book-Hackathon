from openai import OpenAI
from dotenv import load_dotenv
import os

_: bool = load_dotenv()

# Initialize the OpenAI client with Google's Gemini API endpoint
client = OpenAI(
    api_key=os.getenv('GEMINI_API_KEY'),
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)

# Configuration for the agent
class AgentConfig:
    def __init__(self):
        self.model = 'gemini-2.0-flash'
        self.client = client
        self.tracing_disabled = True

config = AgentConfig()