from flask import Flask, render_template, url_for, request, redirect

app = Flask(__name__)

@app.route('/webrtc', methods=['GET', 'POST'])
def webrtc_signaling():
    return 'Received WebRTC signaling:'

@app.route('/')
def index():
    return  render_template('index.html')

if __name__ == "__main__":
    app.run(debug=True)