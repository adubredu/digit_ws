from detecto import core, utils, visualize
from torchvision import transforms

augmentations = transforms.Compose([
    transforms.ToPILImage(),
    transforms.RandomHorizontalFlip(0.5),
    transforms.ColorJitter(saturation=0.5),
    transforms.ToTensor(),
    utils.normalize_transform(),
])

dataset = core.Dataset('../data3/',transform=augmentations)
model = core.Model(['banana', 'cube', 'sponge','lion'])
model.fit(dataset, epochs=100)
model.save('../perception_models/grocery_detector_v4.pth')