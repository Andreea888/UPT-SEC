import subprocess
import re
import requests
import cv2
from pyzbar.pyzbar import decode
import time

TEAM_NAME = "Meeral"
MAX_RETRIES = 5
RETRY_DELAY = 0.3  # seconds between retries

# --- QR scanner ---
def scan_qr():
    print(" Show the QR code to the webcam (ESC to cancel)...")
    cap = cv2.VideoCapture(0)
    url = None
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        for qr in decode(frame):
            url = qr.data.decode("utf-8")
            print(f" QR detected:\n{url}")
            cap.release()
            cv2.destroyAllWindows()
            return url
        cv2.imshow("QR Scanner", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            break
    cap.release()
    cv2.destroyAllWindows()
    return None

# --- Get latest sensor reading from logcat with retries ---
def get_latest_sensor_reading():
    for attempt in range(MAX_RETRIES):
        try:
            # Get last 5 logcat lines with SENSOR tag
            adb_proc = subprocess.Popen(
                ["adb", "logcat", "-t", "5", "-s", "SENSOR"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            stdout, _ = adb_proc.communicate(timeout=2)
            match = re.search(r"temperature=([\d\.]+)\s+humidity=([\d\.]+)", stdout)
            if match:
                temp = float(match.group(1))
                hum = float(match.group(2))
                return temp, hum
        except Exception as e:
            print("Error reading sensor:", e)
        time.sleep(RETRY_DELAY)
    return None, None

# --- Send GET request safely ---
def send_checkin(url, temp, hum):
    url_filled = (
        url
        .replace("YOUR_TEAM", TEAM_NAME)
        .replace("FILL_HERE", f"{temp:.2f}")
        .replace("FILL_THERE", f"{hum:.2f}")
    )
    print(f"→ Sending: {url_filled}")
    headers = {"User-Agent": "Mozilla/5.0"}
    try:
        r = requests.get(url_filled, headers=headers, timeout=5)
        r.raise_for_status()
        try:
            data = r.json()
            print("Server response:", data)
            print("Status:", data.get("status", "UNKNOWN"))
        except ValueError:
            print(" Server did not return valid JSON:")
            print(r.text[:200])
    except requests.exceptions.RequestException as e:
        print(" HTTP error:", e)

# --- Main ---
def main():
    qr_url = scan_qr()
    if not qr_url:
        print("No QR detected, exiting.")
        return

    # Immediately get the latest sensor reading
    temp, hum = get_latest_sensor_reading()
    if temp is None or hum is None:
        print(" Could not read sensor data from Hub after retries.")
        return

    print(f" Temperature: {temp:.2f}°C |  Humidity: {hum:.2f}%")
    send_checkin(qr_url, temp, hum)

if __name__ == "__main__":
    main()

