from setuptools import find_packages, setup


setup(
    name="dual-ov5640-pynq",
    version="2.0.0",
    description="PYNQ stereo SGM, LDS-Net detection/segmentation and HDMI for AX7020",
    package_data={"dual_ov5640": ["liblds_net_post.so", "liblds_net_post.build.json", "assets/ps_ui.npz"]},
    packages=find_packages(),
    python_requires=">=3.6",
)
