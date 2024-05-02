from flask import Flask, request, jsonify
from flask_cors import CORS

app = Flask(__name__)
CORS(app) # allow CORS

# use to receive POST from client and control RPi
@app.route('/control', methods=['POST'])
def control():
    data = request.json
    key = data.get('key')
    
    # According to the key(w,s,d,a) contorl Raspberry Pi
    # Right now, only print
    print(f'Received control key: {key}')
    # Next, focus on 

    # build response message
    response_message = f'Client key received: {key}'
    
    return jsonify({'message': response_message}), 200

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)