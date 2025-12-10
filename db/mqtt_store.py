import sqlite3
import json
from paho.mqtt import client as mqtt

BROKER =  ""
PORT = 8883
TOPIC_EVENT = "/event"
TOPIC_RAW = "/raw_data"

USERNAME = "mqtt_store"
PASSWORD = ""
CA_CERT = "./ca.crt"    # root CA file

conn = sqlite3.connect("data.db")
cur = conn.cursor()


cur.execute("""
CREATE TABLE IF NOT EXISTS event_data (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    device_id INTEGER,
    event TEXT,
    type TEXT,
    direction TEXT,
    value REAL,
    timestamp INTEGER,
    timestring TEXT
)
""")

cur.execute("""
CREATE TABLE IF NOT EXISTS raw_data (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    device_id INTEGER,
    acc_x REAL,
    acc_y REAL,
    acc_z REAL,
    gyro_x REAL,
    gyro_y REAL,
    gyro_z REAL,
    timestamp INTEGER,
    timestring TEXT
)
""")

conn.commit()


def on_connect(client, userdata, flags, rc):
    print("Connected:", rc)
    client.subscribe([(TOPIC_EVENT, 0), (TOPIC_RAW, 0)])


def on_message(client, userdata, msg):
    payload = msg.payload.decode().strip()

    try:
        data = json.loads(payload)
    except:
        return None

    if msg.topic == TOPIC_EVENT:
        cur.execute("""
            INSERT INTO event_data (device_id, event, type, direction, value, timestamp, timestring)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        """, (
            data.get("device_id"),
            data.get("event"),
            data.get("type"),
            data.get("direction"),
            data.get("value"),
            data.get("timestamp"),
            data.get("timestring")
        ))

    elif msg.topic == TOPIC_RAW:
        cur.execute("""
            INSERT INTO raw_data (device_id, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, timestamp, timestring)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            data.get("device_id"),
            data.get("acc_x"),
            data.get("acc_y"),
            data.get("acc_z"),
            data.get("gyro_x"),
            data.get("gyro_y"),
            data.get("gyro_z"),
            data.get("timestamp"),
            data.get("timestring")
        ))

    conn.commit()
    print(f"Stored message from {msg.topic}")

client = mqtt.Client()
client.tls_set(ca_certs=CA_CERT)
client.username_pw_set(USERNAME, PASSWORD)

client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, PORT, 60)
client.loop_forever()
