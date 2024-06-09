
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import binary_erosion
from skimage.filters import threshold_otsu 
from skimage.morphology import remove_small_objects
import torch
import torch.nn as nn

class PathFinder(nn.Module):
    def __init__(self, img_width: int, img_height: int, cropped_img_height: int):
        super().__init__()

        self.img_width = img_width
        self.img_height = img_height
        self.cropped_img_height = cropped_img_height
        
        # features extractor
        self.features = nn.Sequential(
            nn.Conv2d(1, 4, kernel_size=5, stride=2, padding=2),
            nn.ReLU(),
            nn.Conv2d(4, 4, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            
            nn.Conv2d(4, 8, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(8, 8, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            
            nn.Conv2d(8, 16, kernel_size=5, stride=2, padding=2),
            nn.ReLU(),
            nn.Conv2d(16, 16, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
        )
        # classification layers
        self.classifier = nn.Sequential(
            nn.Linear(2624, 1024), # img_height=50 -> 4592; img_height=30 -> 2624
            nn.ReLU(),
            nn.Dropout(.35),
            nn.Linear(1024, 3)
        )

    def forward(self, x: torch.Tensor):
        """Returns the model's predictions for a mini-batch.

        Parameters
        ----------
        x : Tensor
            Mini-batch of input images, grayscale without normalization
        
        Returns
        -------
        Tensor
            Model predictions for given mini-batch
        """
        x = self.features(x)
        #import numpy as np; print(np.prod(x.shape[1:]))
        x = x.view(-1, 2624) # img_height=50 -> 4592; img_height=30 -> 2624
        x = self.classifier(x)
        x = torch.stack([
            torch.sigmoid(x[:, 0]) * (self.img_width-1), # point1_x
            torch.sigmoid(x[:, 1]) * (self.img_width-1), # point2_x
            torch.sigmoid(x[:, 2]) * (self.cropped_img_height-1) # point2_y
        ], dim=1) # apply range constraints
        # rounding to int prevents the model to learn early
        return x

    def load(self, model_path: str):
        """Loads saved weights
        
        Parameters
        ----------
        model_path : str
            Path to the model's saved weights
        """
        try: self.load_state_dict(torch.load(model_path, map_location=torch.device('cpu'))['model_state_dict'])
        except: self.load_state_dict(torch.load(model_path, map_location=torch.device('cpu')))

    def preprocess(self, image, highlight_vert_lines=False, remove_hori_lines=False, plot=False):
        """Preprocess the given image
        
        Parameters
        ----------        highlight_vert_lines : bool

        image : np.array
            Numpy array of float
            True if vertical lines should be highlighted, False otherwise
        remove_hori_lines : bool
            True if horizontal lines should be removed, False otherwise
        
        Returns
        -------
        Torch array of float
        """
        image_preprocessed = self._preprocess(image, highlight_vert_lines, remove_hori_lines)
        # crop
        # Suppose que self.cropped_img_height = 30
        cropped_image = image_preprocessed[:self.cropped_img_height, :].reshape(1, self.cropped_img_height, self.img_width).astype(float)
        cropped_image = torch.from_numpy(cropped_image).to(dtype=torch.float32)
        
        if plot:
            plt.imshow(image_preprocessed, cmap='gray', origin='lower')
            plt.show()
            plt.imshow(cropped_image.numpy()[0], cmap='gray', origin='lower')
            plt.show()
            
        return cropped_image

    def _preprocess(self, image, highlight_vert_lines, remove_hori_lines):
        """Preprocess the given image
        
        Parameters
        ----------
        image : np.array
            Numpy array of float
        highlight_vert_lines : bool
            True if vertical lines should be highlighted, False otherwise
        remove_hori_lines : bool
            True if horizontal lines should be removed, False otherwise
        
        Returns
        -------
        numpy array for the preprocessed image
        """
        if np.max(image) > 1.0:
            image /= 255.0
        
        # perform Otsu thresholding
        thresh = threshold_otsu(image)
        bw = image > thresh
        
        # detect horizontal lines
        if remove_hori_lines:
            horizontal_kernel = np.ones((1, 10), np.uint8)
            detected_hori_lines = binary_erosion(bw, structure=horizontal_kernel, iterations=1)
        
        # detect vertical lines
        if highlight_vert_lines:
            vertical_kernel = np.ones((1, 10), np.uint8).reshape(-1,1)
            detected_vert_lines = binary_erosion(bw, structure=vertical_kernel, iterations=1)
        
        # remove horizontal lines
        if remove_hori_lines:
            bw[detected_hori_lines] = 0
        # highlight vertical lines
        if highlight_vert_lines:
            bw[detected_vert_lines] = 1
        
        # remove small groups of white pixels
        bw = remove_small_objects(bw, min_size=90, connectivity=2)
        #binary_mask_cleaned = remove_small_objects(bw, min_size=1)
        # clear the border to remove any white pixels that are connected to the image border
        #binary_mask_cleaned = clear_border(binary_mask_cleaned)
        #bw[binary_mask_cleaned] = 0
        return bw

    def get_line_coords(self, image):
        """Detect a line and return its coordinates for the given image
        
        Parameters
        ----------
        image : np.array
        
        Returns
        -------
        list of line coordinates: x1,x2,y2
        """
        with torch.no_grad():
            pred = self(image)
            pred = pred.squeeze()
            line_coords = pred.round().cpu().detach().tolist() # /!\ round pre
        return line_coords

    @staticmethod
    def init():
        img_width, img_height = 324, 244
        cropped_img_height = 30
        pth_finder = PathFinder(img_width, img_height, cropped_img_height)
        return pth_finder