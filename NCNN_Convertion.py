from ultralytics import YOLO

model = YOLO("ModelV3.pt")
model.export(format="ncnn")
