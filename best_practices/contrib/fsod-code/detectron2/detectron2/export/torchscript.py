# Copyright (c) Facebook, Inc. and its affiliates. All Rights Reserved
# Copyright 2020 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import importlib.util
import os
import sys
import tempfile
from contextlib import contextmanager
from typing import Dict
import torch

# fmt: off
from detectron2.modeling.proposal_generator import RPN
# need an explicit import due to https://github.com/pytorch/pytorch/issues/38964
from detectron2.structures import Boxes, Instances  # noqa F401

# fmt: on

_counter = 0


def export_torchscript_with_instances(model, fields):
    """
    Run :func:`torch.jit.script` on a model that uses the :class:`Instances` class. Since
    attributes of :class:`Instances` are "dynamically" added in eager mode，it is difficult
    for torchscript to support it out of the box. This function is made to support scripting
    a model that uses :class:`Instances`. It does the following:

    1. Create a scriptable ``new_Instances`` class which behaves similarly to ``Instances``,
       but with all attributes been "static".
       The attributes need to be statically declared in the ``fields`` argument.
    2. Register ``new_Instances`` to torchscript, and force torchscript to
       use it when trying to compile ``Instances``.

    After this function, the process will be reverted. User should be able to script another model
    using different fields.

    Example:
        Assume that ``Instances`` in the model consist of two attributes named
        ``proposal_boxes`` and ``objectness_logits`` with type :class:`Boxes` and
        :class:`Tensor` respectively during inference. You can call this function like:

        ::
            fields = {"proposal_boxes": "Boxes", "objectness_logits": "Tensor"}
            torchscipt_model =  export_torchscript_with_instances(model, fields)

    Args:
        model (nn.Module): The input model to be exported to torchscript.
        fields (Dict[str, str]): Attribute names and corresponding type annotations that
            ``Instances`` will use in the model. Note that all attributes used in ``Instances``
            need to be added, regarldess of whether they are inputs/outputs of the model.
            Custom data type is not supported for now.

    Returns:
        torch.jit.ScriptModule: the input model in torchscript format
    """
    with patch_instances(fields):

        # Also add some other hacks for torchscript:
        # boolean as dictionary keys is unsupported:
        # https://github.com/pytorch/pytorch/issues/41449
        # We annotate it this way to let torchscript interpret them as integers.
        RPN.__annotations__["pre_nms_topk"] = Dict[int, int]
        RPN.__annotations__["post_nms_topk"] = Dict[int, int]

        scripted_model = torch.jit.script(model)
        return scripted_model


@contextmanager
def patch_instances(fields):
    with tempfile.TemporaryDirectory(prefix="detectron2") as dir, tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", suffix=".py", dir=dir, delete=False
    ) as f:
        try:
            cls_name, s = _gen_module(fields)
            f.write(s)
            f.flush()
            f.close()

            module = _import(f.name)
            new_instances = getattr(module, cls_name)
            _ = torch.jit.script(new_instances)

            # let torchscript think Instances was scripted already
            Instances.__torch_script_class__ = True
            # let torchscript find new_instances when looking for the jit type of Instances
            Instances._jit_override_qualname = torch._jit_internal._qualified_name(new_instances)
            yield new_instances
        finally:
            try:
                del Instances.__torch_script_class__
                del Instances._jit_override_qualname
            except AttributeError:
                pass
            sys.modules.pop(module.__name__)


# TODO: find a more automatic way to enable import of other classes
def _gen_imports():
    imports_str = """
import torch
from torch import Tensor
import typing
from typing import *

from detectron2.structures import Boxes

"""
    return imports_str


def _gen_class(fields):
    def indent(level, s):
        return " " * 4 * level + s

    lines = []

    global _counter
    _counter += 1

    cls_name = "Instances_patched{}".format(_counter)

    lines.append(
        f"""
class {cls_name}:
    def __init__(self, image_size: Tuple[int, int]):
        self.image_size = image_size
"""
    )

    for name, type_ in fields.items():
        lines.append(indent(2, f"self.{name} = torch.jit.annotate(Optional[{type_}], None)"))
    # TODO add getter/setter when @property is supported

    return cls_name, os.linesep.join(lines)


def _gen_module(fields):
    s = ""
    s += _gen_imports()
    cls_name, cls_def = _gen_class(fields)
    s += cls_def
    return cls_name, s


def _import(path):
    # https://docs.python.org/3/library/importlib.html#importing-a-source-file-directly
    spec = importlib.util.spec_from_file_location(
        "{}{}".format(sys.modules[__name__].__name__, _counter), path
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[module.__name__] = module
    spec.loader.exec_module(module)
    return module
