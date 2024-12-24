from ultralytics import YOLO

model = YOLO("yolov8n.pt") 

model.train(
    data="/home/pratama/Desktop/Code1/TubesMikom/Uno-Cards-1/data.yaml", 
    epochs=15,                  # Jumlah epoch
    imgsz=640,                  # Image size
    batch=16,                   # Batch size
    name="Model_unoCard",       # Folder output
    project="runs/train",       # Output hasil training
    device= -1                  # GPU (0) atau CPU (-1)
)
