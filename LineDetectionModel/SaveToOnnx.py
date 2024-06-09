import torch
import torch.onnx
from PathFinder import PathFinder
import os
from PIL import Image
import numpy as np

target_width, target_height = 324, 244

# Initialisation du modèle
model = PathFinder.init()

# Poids du modèle
model_path = './pathfinder3.pth'

# Chargement des poids dans le modèle
model.load(model_path)

# Définir un tenseur d'entrée
dummy_input = torch.randn(1, 1, model.cropped_img_height, model.img_width)

# Chemin du onnx file de notre modèle
onnx_path = 'pathfinder3.onnx'

# Export du modèle en onnx
torch.onnx.export(model, dummy_input, onnx_path, export_params=True, opset_version=10,
                  do_constant_folding=True, input_names=['input'], output_names=['output'],
                  dynamic_axes={'input': {0: 'batch_size'}, 'output': {0: 'batch_size'}})

print(f'Model has been converted to ONNX and saved at {onnx_path}')

def save_tensor_as_png(tensor, path):
    # Supprimer l'axe de canal
    tensor = tensor.squeeze(0)
    # Convertir le tensor PyTorch en tableau NumPy
    image_array = tensor.numpy()
    # Normaliser les valeurs en float dans la plage [0, 255]
    image_array = (image_array - image_array.min()) / (image_array.max() - image_array.min()) * 255.0
    # Convertir le tableau NumPy en image PIL
    image = Image.fromarray(np.uint8(image_array)).rotate(180)
    # Sauvegarder l'image PIL en PNG
    image.save(path, 'PNG')

# Directory contenant les images
input_directory = ['drone_images/', 'render/']
output_directory = 'drone_images_cropped/'

# Créer le répertoire de sortie s'il n'existe pas
os.makedirs(output_directory, exist_ok=True)

# Itérer sur toutes les images dans le répertoire
for dir in input_directory:
    for filename in os.listdir(dir):
        if filename.endswith('.png') or filename.endswith('.jpg') or filename.endswith('.jpeg'):
            # Charger l'image en niveaux de gris
            image_path = os.path.join(dir, filename)
            image = Image.open(image_path).convert('L').resize((target_width, target_height), Image.Resampling.LANCZOS).rotate(180)
            image_array = np.array(image).astype(np.float32)

            image_tensor = model.preprocess(image_array)

            # Sauvegarder le tensor en tant qu'image PNG
            output_path = os.path.join(output_directory, os.path.splitext(filename)[0] + '.png')
            save_tensor_as_png(image_tensor, output_path)

print("Traitement terminé.")


