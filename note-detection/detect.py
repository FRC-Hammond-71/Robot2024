# Lint as: python3
# Copyright 2019 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
r"""Example using PyCoral to detect objects in a given image.

To run this code, you must attach an Edge TPU attached to the host and
install the Edge TPU runtime (`libedgetpu.so`) and `tflite_runtime`. For
device setup instructions, see coral.ai/docs/setup.

Example usage:
```
bash examples/install_requirements.sh detect_image.py

python3 examples/detect_image.py \
  --model test_data/ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite \
  --labels test_data/coco_labels.txt \
  --input test_data/grace_hopper.bmp \
  --output ${HOME}/grace_hopper_processed.bmp
```
"""

import argparse
import time
from webcam import Webcam

import cv2

from pycoral.adapters import common
from pycoral.adapters import detect
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter

def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "-m", "--model", required=True, help="File path of .tflite file"
    )
    # parser.add_argument(
    #     "-i", "--input", required=True, help="File path of image to process"
    # )
    parser.add_argument("-l", "--labels", help="File path of labels file")
    parser.add_argument(
        "-t",
        "--threshold",
        type=float,
        default=0.4,
        help="Score threshold for detected objects",
    )
    parser.add_argument("-f", "--fps", help="Amount of times the frame is refreshed per second", default=5, type=int)
    # parser.add_argument(
    #     "-o", "--output", help="File path for the result image with annotations"
    # )
    parser.add_argument(
        "-c", "--count", type=int, default=5, help="Number of times to run inference"
    )
    args = parser.parse_args()

    labels = read_label_file(args.labels) if args.labels else {}
    print("Setting up interpreter...")
    interpreter = make_interpreter(args.model)
    print("Allocating tensors...")
    interpreter.allocate_tensors()

    size = common.input_size(interpreter)
    assert size != None
    webcam = Webcam(src=0, w=size[0], h=size[1])
    print("Webcam created")

    lastupdate = time.time()
    for frame in webcam:
        inference_time = 0;
        if time.time() - lastupdate < args.fps / 60:
            # Skip frames
            continue

        for _ in range(args.count):
            start = time.perf_counter()
            common.set_input(interpreter, frame.data)
            interpreter.invoke()
            inference_time += time.perf_counter() - start
            objs = detect.get_objects(interpreter, args.threshold, (1, 1))

        if objs:
            for obj in objs:
                cv2.rectangle(
                    frame,
                    (obj.bbox.xmin, obj.bbox.ymin),
                    (obj.bbox.xmax, obj.bbox.ymax),
                    (255, 0, 0),
                    2,
                )

                cv2.putText(
                    frame,
                    "{id}: {label} {score:.2f}".format(label=labels.get(obj.id, obj.id), score=obj.score, id=obj.id),
                    (obj.bbox.xmin + 10, obj.bbox.ymin - 15),
                    cv2.FONT_HERSHEY_PLAIN,
                    1,
                    (255, 0, 0)
                )

        cv2.putText(
            frame,
            "{time:.2f} s {fps:.2f} fps".format(time=inference_time, fps=args.fps),
            (10, 10),
            cv2.FONT_HERSHEY_PLAIN,
            0.8,
            (255, 0, 0)
        )

        cv2.imshow("Note Detection", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

        # Break the loop if the user presses the 'q' key
        if cv2.waitKey(1) & 0xFF == ord("q"):
          return

        lastupdate = time.time()

if __name__ == "__main__":
    main()
