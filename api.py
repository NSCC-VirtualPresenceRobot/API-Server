from flask import Flask, request, jsonify
from flask_cors import CORS
from robot_control import RobotControl
import threading

app = Flask(__name__)
CORS(app) # allow CORS
robot = RobotControl()

# use to receive POST from client and control RPi
@app.route('/control', methods=['POST'])
def control():
    data = request.json
    key = data.get('key')
    
    # According to the key(w,s,d,a) contorl Raspberry Pi
    # Right now, only print
    print(f'Received control key: {key}')
    # Next, focus on real control
    if key == 'w+a':
        robot.diagonal_up_left()
    elif key == 'w+d':
        robot.diagonal_up_right()
    elif key == 's+a':
        robot.diagonal_back_left()
    elif key == 's+d':
        robot.diagonal_back_right()
    elif key == 'w+j':
        robot.forward_left()
    elif key == 'w+k':
        robot.forward_right()
    elif key == 'w':
        robot.forward()
    elif key == 's':
        robot.backward()
    elif key == 'a':
        robot.turn_left() 
    elif key == 'd':
        robot.turn_right()
    elif key == 'j':
        robot.strafe_left() # change later
    elif key == 'k':
        robot.strafe_right() # change later
    elif key == 'stop':
        robot.stop()

    # build response message
    response_message = f'Client key received: {key}'
    
    return jsonify({'message': response_message}), 200

def run_robot_threads():
    movement_thread = threading.Thread(target=robot.handle_movement)
    movement_thread.start()

    lcd_thread = threading.Thread(target=robot.update_lcd)
    lcd_thread.start()

    return movement_thread, lcd_thread

if __name__ == '__main__':
    # run robot threads before run api server
    movement_thread, lcd_thread = run_robot_threads()

    app.run(debug=True, host='0.0.0.0', port=5000)

    # When the application exits, stop the threads
    robot.stop_threads()
    movement_thread.join()
    lcd_thread.join()