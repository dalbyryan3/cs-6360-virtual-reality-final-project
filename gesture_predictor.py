# For reference:
# Label 1 = zero-based label 0 = up
# Label 2 = zero-based label 1 = right
# Label 3 = zero-based label 2 = down
# Label 4 = zero-based label 3 = left
# Label 5 = zero-based label 4 = circle
# Label 6 = zero-based label 5 = x

import serial
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import torch
from training.train_gesture_classification import GestureClassificationNet, read_and_interpolate_features
from training.create_training_data import next_vals, default_vals, save_csv
from flask import Flask, json, jsonify
import time
import threading
import logging

def make_prediction(model, features):
    model.eval()
    with torch.no_grad():
        score = model(features)
    return torch.argmax(score, dim=1).cpu().item()
def prediction_loop(current_prediction_dict, model, port, baud_rate, root_dir, device, dtype, sample_size_min=20):
    vr_handheld_controller_receiver = serial.Serial(port=port, baudrate=baud_rate)
    val_list = [''] * 9 # acc_x, acc_y, acc_z, gyr_x, gyr_y, gry_z, mag_x, mag_y, mag_z
    temp_filepath = os.path.join(root_dir,'current_reading_tempfile.csv')
    while True:
        try:
            msg = vr_handheld_controller_receiver.read_until()
            msg = msg.decode('utf-8')
            if msg is None or msg == '':
                # print('msg is none or empty str')
                continue
            msg_list = msg.rstrip('\r\n').split(';')
            if len(msg_list) != 10:
                print(msg)
                continue
            if msg_list[0] == '1':
                val_list = next_vals(val_list, msg_list)
            else:
                if len(val_list[0])  > sample_size_min:
                    save_csv(temp_filepath, val_list)
                    features = torch.unsqueeze(read_and_interpolate_features(temp_filepath),dim=0)
                    features = features.to(device=device, dtype=dtype)
                    os.remove(temp_filepath)
                    current_prediction_dict['currentprediction'] = make_prediction(model, features)
                    print('Gesture detected, predicted gesture = {0}'.format(current_prediction_dict['currentprediction']))
                val_list = default_vals()
        except:
            print('Connection ended')
            vr_handheld_controller_receiver.close()
            time.sleep(5)
            vr_handheld_controller_receiver = serial.Serial(port=port, baudrate=baud_rate)


current_prediction_dict = {'currentprediction':-1}

server_ip = '127.0.0.1'
server_port = 5151

app = Flask(__name__)

log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

@app.route("/", methods=['GET'])
def home():
    return "<p>VR Handheld Controller Gesture Predictor API</p>"
@app.route("/currentprediction", methods=['GET'])
def give_current_prediction():
    global current_prediction_dict
    current_prediction_json = jsonify(current_prediction_dict)
    current_prediction_json.headers.add('Access-Control-Allow-Origin', '*')
    return current_prediction_json

def run_app_server():
    app.run(debug=False, threaded=True, host=server_ip, port=server_port)
def gesture_prediction_loop():
    global current_prediction_dict
    training_root_dir = './training'
    models_path = './training/models'
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    dtype = torch.float32
    prediction_model_filename = 'gesture_model_12_12_2021_14_15_54.pt' 
    prediction_model_filepath = os.path.join(models_path, prediction_model_filename)

    gesture_classification_model = GestureClassificationNet().to(device=device, dtype=dtype)
    gesture_classification_model.load_state_dict(torch.load(prediction_model_filepath))

    port = '/dev/tty.usbmodem11301'
    baud_rate = 15200
    sample_size_min = 20
    prediction_loop(current_prediction_dict, gesture_classification_model, port, baud_rate, training_root_dir, device=device, dtype=dtype, sample_size_min=sample_size_min)

if __name__=="__main__":
    app_server_thread = threading.Thread(target=run_app_server)
    test_thread = threading.Thread(target=gesture_prediction_loop)
    app_server_thread.start()
    test_thread.start()
