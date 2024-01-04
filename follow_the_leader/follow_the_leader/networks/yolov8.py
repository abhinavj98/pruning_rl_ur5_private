from ultralytics import YOLO
import numpy as np
from ultralytics.utils.ops import scale_image
import matplotlib.pyplot as plt

class YoloInference:
    def __init__(self, input_size=(640, 448), output_size=(640, 448),
                    model_path='yolov5s.pt'):
        self.input_size = input_size
        self.output_size = output_size

        self.model = YOLO(model_path)  # load a pretrained YOLOv8 segmentation model
        print("Model weights loaded")

    # def preprocess(self, image):

    def predict(self, image):
        """predicting image
        """
        from PIL import Image
        import torch
        result = self.model(source=image)[0]
        # for result in results:
        masks = result.masks.data
        cls = result.boxes.data #bbox, (N, 6)
        cls = cls[:, 5] #Class valye
        print(cls)
        people_indices = torch.where(cls == 0)
        print('people_indices', people_indices)
        # use these indices to extract the relevant masks
        people_masks = masks[people_indices]
        print('people_masks', people_masks.shape)
        # scale for visualizing results
        people_mask = torch.any(people_masks, dim=0).int() * 255
        print(people_mask.shape)

        cv2.imwrite(str('/Users/abhinav/Desktop/gradstuff/research/pruning_control_ur5/follow_the_leader/follow_the_leader/networks/abc.jpg'), people_mask.cpu().numpy())
        # masks = np.moveaxis(masks, 0, -1)
        # print(masks)
        # masks = scale_image(masks.shape[:2], masks, result.masks.orig_shape)
        # masks = np.array(masks, dtype=bool)
        # cls = result.boxes.data.cpu().numpy()  # cls, (N, 1)

        # xy = np.array(xy)
        # #move axis
        # # xy = np.moveaxis(xy, 0, -1)
        # # image = Image.fromarray(image)
        # print(xy.shape, image.shape)
        # for x, y in xy:
        #     image[x, y] =
        # image[xy] = 255
        im_array = result.plot(font_size=7, line_width=3)
        # im_array = r.plot()  # plot a BGR numpy array of predictions
        # im = Image.fromarray(im_array[..., ::-1])  # RGB PIL image
        # im.show()  # show ima
        # masks = result.masks.masks.cpu().numpy()  # masks, (N, H, W)
        # masks = np.moveaxis(masks, 0, -1)  # masks, (H, W, N)
        # masks = scale_image(masks.shape[:2], masks, result.masks.orig_shape)
        # masks = np.array(masks, dtype=bool)
        # cls = result.boxes.cls.cpu().numpy()  # cls, (N, 1)
        # probs = result.boxes.conf.cpu().numpy()  # confidence score, (N, 1)

        return

if __name__ == "__main__":
    model_path = 'best.pt'
    yolo = YoloInference(model_path=model_path)
    #load image using cv2
    import cv2
    # image = cv2.imread('/Users/abhinav/Downloads/IMG_7794.JPG')
    image = cv2.imread('bus.jpg')
    yolo.predict(image)