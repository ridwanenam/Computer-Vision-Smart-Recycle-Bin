from ultralytics import YOLO

# Load model YOLOv8 yang sudah pre-trained
model = YOLO("yolov8n.pt")  # Anda juga bisa mengganti dengan 'yolov8s.pt', dll.

# Melatih model
model.train(
    data="/home/pratama/Desktop/Code1/TubesMikom/Uno-Cards-1/data.yaml",  # Path ke file data.yaml
    epochs=15,                  # Jumlah epoch
    imgsz=640,                  # Ukuran gambar
    batch=16,                   # Batch size
    name="Model_unoCard",   # Nama folder output
    project="runs/train",       # Lokasi hasil training
    device= -1                    # Gunakan GPU (0) atau CPU (-1)
)
