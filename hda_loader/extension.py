# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.ext

EXTENSION_NAME = "HDA Loader"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        pass

    def on_shutdown(self):
        pass
