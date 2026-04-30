#! /usr/bin/env python3
#
# PyMatcher script for rtabmap using cvg/LightGlue (SuperPoint features).
# To use with rtabmap:
#   --Vis/CorNNType 6 \
#   --PyMatcher/Path "~/rtabmap_ws/src/rtabmap/corelib/src/python/rtabmap_lightglue.py"
#
# Requires: pip install --user /tmp/LightGlue (cvg/LightGlue from GitHub)
#

import numpy as np
import torch

from lightglue import LightGlue

torch.set_grad_enabled(False)

device = 'cpu'
matcher = None


def init(descriptorDim, matchThreshold, iterations, cuda, model):
    print("LightGlue python init()")
    global device, matcher
    device = 'cuda' if torch.cuda.is_available() and cuda else 'cpu'
    # `model` is reused from the SuperGlue interface ('indoor'/'outdoor') —
    # LightGlue's SuperPoint weights are scene-agnostic, so we ignore it.
    matcher = LightGlue(
        features='superpoint',
        depth_confidence=-1 if iterations <= 0 else 0.95,
        width_confidence=-1 if iterations <= 0 else 0.99,
        filter_threshold=float(matchThreshold),
    ).eval().to(device)


def match(kptsFrom, kptsTo, scoresFrom, scoresTo,
          descriptorsFrom, descriptorsTo, imageWidth, imageHeight):
    global device, matcher

    # When called from embedded C++ (PyArray_SimpleNewFromData), the arrays
    # are a "foreign" numpy.ndarray that torch.from_numpy() rejects.
    # Use torch.as_tensor() which accepts any array-like via __array__ protocol.
    kptsFrom_t = torch.as_tensor(kptsFrom, dtype=torch.float32).to(device)
    kptsTo_t = torch.as_tensor(kptsTo, dtype=torch.float32).to(device)
    descFrom_t = torch.as_tensor(descriptorsFrom, dtype=torch.float32).to(device)
    descTo_t = torch.as_tensor(descriptorsTo, dtype=torch.float32).to(device)

    if kptsFrom_t.shape[0] < 2 or kptsTo_t.shape[0] < 2:
        return np.empty((0, 2), dtype=np.int64)

    data = {
        'image0': {
            'keypoints': kptsFrom_t.unsqueeze(0),
            'descriptors': descFrom_t.unsqueeze(0),
            'image_size': torch.tensor([[imageWidth, imageHeight]],
                                       dtype=torch.float32, device=device),
        },
        'image1': {
            'keypoints': kptsTo_t.unsqueeze(0),
            'descriptors': descTo_t.unsqueeze(0),
            'image_size': torch.tensor([[imageWidth, imageHeight]],
                                       dtype=torch.float32, device=device),
        },
    }

    out = matcher(data)
    # out['matches'] is a list of (M, 2) int tensors, one per pair.
    m = out['matches'][0].detach().cpu()
    # Convert via tolist() + np.array to avoid cross-numpy-ABI issues
    # in embedded Python. The C++ side expects NPY_INT32 (type=5).
    m_list = m.tolist()
    matches = np.array(m_list, dtype=np.int32)
    if matches.ndim == 1:
        matches = matches.reshape(-1, 2)
    return matches


if __name__ == '__main__':
    init(256, 0.2, 20, True, 'indoor')
    rng = np.random.default_rng(0)
    kp0 = rng.random((50, 2)) * 512
    kp1 = rng.random((50, 2)) * 512
    d0 = rng.standard_normal((50, 256))
    d1 = rng.standard_normal((50, 256))
    s0 = rng.random(50)
    s1 = rng.random(50)
    m = match(kp0, kp1, s0, s1, d0, d1, 512, 512)
    print("matches:", m.shape)
