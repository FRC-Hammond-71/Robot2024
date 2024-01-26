# Note Detector

## Getting Setup

1. [Install Google Coral EDGE TPU Runtime](https://coral.ai/docs/accelerator/get-started/#runtime-on-windows)

1. Use Python 3.9

```
python3.9 -m pip install -r requirements.txt
```

```
python3.9 -m pip install --extra-index-url https://google-coral.github.io/py-repo/ pycoral
```

## Running

```
python3.9 detect.py --model ./data/edgetpu.tflite --labels ./data/labelmap.txt --count 2
```
