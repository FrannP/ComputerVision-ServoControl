import cv2  # inicijaliziranje biblioteka: cv2, serial, time
import serial
import time

# Inicijaliziranje serijske komunikacije s arduinom
arduino_port = 'COM5'
baud_rate = 9600
ser = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)

# inicijaliziranje klasifikatora za detekciju lica
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Inicijaliziranje pozicije servo motora
# servo (360 stupnjeva) po x osi
middle_position_x = 90
left_limit_x = 0
right_limit_x = 180
# servo (continuous) po y osi
middle_position_y = 92
up_limit_y = 85
down_limit_y = 97

# Funkcija za mapiranje pozicije lica na kut serva za x-os (pan)
def map_face_position_x(x, width):
    # Mapiranje x-koordinate na kut serva za x-os (pan)
    return int((x / width) * (right_limit_x - left_limit_x) + left_limit_x)

# Funkcija za mapiranje pozicije lica na kut serva za y-os (tilt)
def map_face_position_y(y, height):
    if y < height // 3:
        return up_limit_y
    elif y > 2 * height // 3:
        return down_limit_y
    else:
        return middle_position_y

# Otvaranje kamere
cap = cv2.VideoCapture(0)

# Ogledanje slike horizontalno
def mirror_frame(frame):
    return cv2.flip(frame, 1)

# Veličina koraka za prilagodbu serva
step_size_x = 5  # Podesi po potrebi
error_margin = 15  # Povećaj marginu pogreške
delay_x = 0.01 # Kašnjenje u sekundama za sporiju prilagodbu

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Ogledanje slike horizontalno
    frame = mirror_frame(frame)

    # Dodavanje statične linije kroz sredinu prozora za x-os
    line_position_x = (frame.shape[1] // 2, 0)
    cv2.line(frame, line_position_x, (line_position_x[0], frame.shape[0]), (0, 255, 0), 2)

    # Dodavanje statične linije kroz sredinu prozora za y-os
    line_position_y = (0, frame.shape[0] // 2)
    cv2.line(frame, line_position_y, (frame.shape[1], line_position_y[1]), (0, 255, 0), 2)

    # Pretvaranje slike u sivo za bržu obradu
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detekcija lica na slici
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)

    if len(faces) == 0:
        # Ako lice nije detektirano, postavi servo na srednju poziciju za obje osi
        ser.write((str(middle_position_x) + ',' + str(middle_position_y) + '\n').encode())
    else:
        for (x, y, w, h) in faces:
            # Nacrtaj pravokutnik oko detektiranog lica
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

            # Izračunaj poziciju lica u odnosu na širinu slike za x-os (pan)
            face_position_x = x + w // 2
            target_position_x = map_face_position_x(face_position_x, frame.shape[1])

            # Postupno prilagođavaj middle_position_x prema target_position_x s marginom pogreške
            if target_position_x - error_margin <= middle_position_x <= target_position_x + error_margin:
                # Ako je trenutna pozicija unutar margine pogreške, ne prilagođavaj
                pass
            elif middle_position_x < target_position_x:
                middle_position_x += step_size_x
                time.sleep(delay_x)  # Uvedi kašnjenje
            elif middle_position_x > target_position_x:
                middle_position_x -= step_size_x
                time.sleep(delay_x)  # Uvedi kašnjenje

            # Izračunaj poziciju lica u odnosu na visinu slike za y-os (tilt)
            face_position_y = y + h // 2
            target_position_y = map_face_position_y(face_position_y, frame.shape[0])

            # Ispiši y-poziciju u terminal
            print("Y Position:", target_position_y)

            # Šalje kut servo motora arduinu
            ser.write((str(middle_position_x) + ',' + str(target_position_y) + '\n').encode())

    # Prikazuje prozor
    cv2.imshow('Face Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        # Postavlja servo motore na središnju vrijednost nakon klika na "q"
        ser.write((str(middle_position_x) + ',' + str(middle_position_y) + '\n').encode())
        break

# Prekid veze s kamerom, gasi prozore i prestaje serijsku komunikaciju
cap.release()
cv2.destroyAllWindows()
ser.close()
