import torch as th
from torchvision.models.optical_flow import Raft_Small_Weights, raft_small
from torchvision.transforms import functional as F


class RAFT:
    def __init__(self, size = (224, 224)):
        self.device = "cuda" if th.cuda.is_available() else "cpu"
        weights = Raft_Small_Weights.DEFAULT
        self.transforms = weights.transforms()
        model = raft_small(weights=Raft_Small_Weights.DEFAULT, progress=False).to(self.device)
        self.model = model.eval()
        self.size = size

    def _preprocess(self, img1, img2):

        img1 = F.resize(img1, size=self.size, antialias=False)
        img2 = F.resize(img2, size=self.size, antialias=False)
        return self.transforms(img1, img2)

    def forward(self, current_rgb, previous_rgb):
        current_rgb, previous_rgb = self._preprocess(th.tensor(current_rgb).permute(2, 0, 1).unsqueeze(0),
                                                     th.tensor(previous_rgb).permute(2, 0, 1).unsqueeze(0))
        with th.no_grad():
            list_of_flows = self.model(current_rgb.to(self.device), previous_rgb.to(self.device))
        predicted_flows = list_of_flows[-1]
        predicted_flows[:, 0, :, :] /= self.size[0]
        predicted_flows[:, 1, :, :] /= self.size[1]

        # from torchvision.utils import flow_to_image
        # print(predicted_flows.shape, predicted_flows.max(), predicted_flows.min())
        #
        # flow_img = flow_to_image(predicted_flows)
        # print(flow_img.shapee, flow_img.max(), flow_img.min())
        return predicted_flows[0].cpu().numpy()
