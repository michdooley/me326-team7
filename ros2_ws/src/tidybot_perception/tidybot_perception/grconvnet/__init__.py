"""
Vendored GR-ConvNet (Generative Residual Convolutional Neural Network)

Lightweight CNN for antipodal grasp detection from depth/RGB-D images.
Predicts per-pixel grasp quality, angle, and width.

Original: https://github.com/skumra/robotic-grasping (IROS 2020)
Paper: "Antipodal Robotic Grasping using Generative Residual Convolutional Neural Network"
       by Sulabh Kumra, Shirin Joshi, Ferat Sahin

Vendored to avoid heavy dependency; only model + post-processing included.
"""

from tidybot_perception.grconvnet.grconvnet import GenerativeResnet
from tidybot_perception.grconvnet.post_process import post_process_output, detect_grasps

__all__ = ['GenerativeResnet', 'post_process_output', 'detect_grasps']
