import os

import tensorflow as tf
from tensorflow import keras
import tensorflow_hub as hub
import numpy as np

print(tf.__version__)


class TFmodel:
    def __init__(self):
        self.path = '../Objekt_detektion_model/textu/my_model.h5'
        self.model = tf.keras.models.load_model(self.path, custom_objects={'KerasLayer': hub.KerasLayer})

        self.model.summary()

    def predict_from_image(self, image):
        image = tf.image.convert_image_dtype(image, dtype=tf.float32)
        tfimage = np.expand_dims(image, axis=0)
        return self.model.predict(tfimage)
