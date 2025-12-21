from flask import Flask, request, jsonify, render_template_string
from flask_cors import CORS
from rag_bot import RAGBot
import os
from dotenv import load_dotenv
import logging

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# Initialize the RAG bot
rag_bot = RAGBot(collection_name="teacher_embedding")

# Check if collection exists
collection_exists = rag_bot.check_collection_exists()
if not collection_exists:
    logger.warning("Collection 'teacher_embedding' does not exist. Please run the embedding pipeline first.")

# HTML template for the chat interface
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Documentation RAG Chat</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            max-width: 800px;
            margin: 0 auto;
            padding: 20px;
            background-color: #f5f5f5;
        }
        .chat-container {
            background-color: white;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            margin-bottom: 20px;
        }
        .message {
            margin: 10px 0;
            padding: 10px;
            border-radius: 5px;
        }
        .user-message {
            background-color: #e3f2fd;
            text-align: right;
        }
        .bot-message {
            background-color: #f5f5f5;
        }
        .input-container {
            display: flex;
            flex-direction: column;
            gap: 10px;
        }
        .query-controls {
            display: flex;
            gap: 10px;
        }
        #user-input {
            padding: 10px;
            border: 1px solid #ddd;
            border-radius: 5px;
        }
        #provider-select {
            padding: 10px;
            border: 1px solid #ddd;
            border-radius: 5px;
            background-color: white;
        }
        #send-btn {
            padding: 10px 20px;
            background-color: #2196f3;
            color: white;
            border: none;
            border-radius: 5px;
            cursor: pointer;
        }
        #send-btn:hover {
            background-color: #1976d2;
        }
        #send-btn:disabled {
            background-color: #bbdefb;
            cursor: not-allowed;
        }
        .sources {
            font-size: 0.8em;
            color: #666;
            margin-top: 5px;
        }
        .typing-indicator {
            color: #666;
            font-style: italic;
        }
        .provider-info {
            font-size: 0.8em;
            color: #666;
            margin-top: 5px;
        }
    </style>
</head>
<body>
    <h1>Documentation RAG Chat</h1>
    <div class="chat-container" id="chat-container">
        <div class="message bot-message">
            Hello! I'm your documentation assistant. Ask me anything about the content in the documentation.
            {% if not collection_exists %}
            <div style="color: red; margin-top: 10px;">
                <strong>Warning:</strong> No embedding collection found. Please run the embedding pipeline first.
            </div>
            {% endif %}
        </div>
    </div>
    <div class="input-container">
        <div class="query-controls">
            <input type="text" id="user-input" placeholder="Ask a question about the documentation..." onkeypress="handleKeyPress(event)">
            <select id="provider-select">
                <option value="cohere">Cohere</option>
                <option value="openai">OpenAI</option>
                <option value="gemini">Gemini</option>
            </select>
            <button id="send-btn" onclick="sendMessage()">Send</button>
        </div>
        <div class="provider-info">Current provider: <span id="current-provider">Cohere</span></div>
    </div>

    <script>
        function handleKeyPress(event) {
            if (event.key === 'Enter') {
                sendMessage();
            }
        }

        function updateCurrentProvider() {
            const providerSelect = document.getElementById('provider-select');
            const currentProviderSpan = document.getElementById('current-provider');
            currentProviderSpan.textContent = providerSelect.options[providerSelect.selectedIndex].text;
        }

        // Initialize provider display
        updateCurrentProvider();

        // Add event listener for provider change
        document.getElementById('provider-select').addEventListener('change', updateCurrentProvider);

        function sendMessage() {
            const input = document.getElementById('user-input');
            const message = input.value.trim();
            const providerSelect = document.getElementById('provider-select');
            const provider = providerSelect.value;

            if (!message) return;

            // Add user message to chat
            addMessage(message, 'user');
            input.value = '';
            document.getElementById('send-btn').disabled = true;

            // Show typing indicator
            const typingId = addTypingIndicator();

            // Send message to backend
            fetch('/chat', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({query: message, provider: provider})
            })
            .then(response => response.json())
            .then(data => {
                // Remove typing indicator
                removeTypingIndicator(typingId);

                // Add bot response
                addMessage(data.answer, 'bot', data.sources);

                // Update current provider display to show what was actually used
                if (data.provider) {
                    const currentProviderSpan = document.getElementById('current-provider');
                    currentProviderSpan.textContent = data.provider.charAt(0).toUpperCase() + data.provider.slice(1);
                }
            })
            .catch(error => {
                // Remove typing indicator
                removeTypingIndicator(typingId);

                // Add error message
                addMessage('Sorry, I encountered an error. Please try again.', 'bot');
                console.error('Error:', error);
            })
            .finally(() => {
                document.getElementById('send-btn').disabled = false;
            });
        }

        function addMessage(text, sender, sources = null) {
            const chatContainer = document.getElementById('chat-container');
            const messageDiv = document.createElement('div');
            messageDiv.className = `message ${sender}-message`;

            if (sender === 'user') {
                messageDiv.textContent = text;
            } else {
                messageDiv.innerHTML = text;

                if (sources && sources.length > 0) {
                    const sourcesDiv = document.createElement('div');
                    sourcesDiv.className = 'sources';
                    sourcesDiv.innerHTML = '<strong>Sources:</strong> ' + sources.slice(0, 3).map(src =>
                        `<a href="${src}" target="_blank">${src}</a>`
                    ).join(', ');
                    messageDiv.appendChild(sourcesDiv);
                }
            }

            chatContainer.appendChild(messageDiv);
            chatContainer.scrollTop = chatContainer.scrollHeight;
        }

        function addTypingIndicator() {
            const chatContainer = document.getElementById('chat-container');
            const typingDiv = document.createElement('div');
            typingDiv.className = 'message bot-message typing-indicator';
            typingDiv.id = 'typing-' + Date.now();
            typingDiv.textContent = 'Thinking...';
            chatContainer.appendChild(typingDiv);
            chatContainer.scrollTop = chatContainer.scrollHeight;
            return typingDiv.id;
        }

        function removeTypingIndicator(typingId) {
            const typingElement = document.getElementById(typingId);
            if (typingElement) {
                typingElement.remove();
            }
        }
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    """Serve the chat interface"""
    return render_template_string(HTML_TEMPLATE, collection_exists=collection_exists)

@app.route('/chat', methods=['POST'])
def chat():
    """Handle chat requests"""
    try:
        data = request.json
        query = data.get('query', '')
        provider = data.get('provider', 'gemini')  # Default to gemini if not specified

        if not query:
            return jsonify({'error': 'Query is required'}), 400

        # Validate provider - only allow gemini
        if provider != 'gemini':
            provider = 'gemini'  # Default to gemini if invalid

        # Get response from RAG bot
        result = rag_bot.answer_query(query, top_k=3, provider=provider)

        return jsonify({
            'answer': result['answer'],
            'sources': result['sources'],
            'provider': provider
        })

    except Exception as e:
        logger.error(f"Error processing chat request: {str(e)}")
        return jsonify({'error': 'Internal server error'}), 500

@app.route('/health')
def health():
    """Health check endpoint"""
    return jsonify({'status': 'healthy', 'collection_exists': collection_exists})

if __name__ == '__main__':
    port = int(os.environ.get('PORT', 5000))
    app.run(host='0.0.0.0', port=port, debug=True)