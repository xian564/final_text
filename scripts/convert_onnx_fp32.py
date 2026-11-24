#!/usr/bin/env python3
"""把 armor.onnx 从 FP16 转成 ORT(CPU) 可加载的 FP32/opset19/IR9 模型。
用法: python3 convert_onnx_fp32.py in.onnx out.onnx
"""
import sys
import onnx
from onnx import numpy_helper, version_converter

src, dst = sys.argv[1], sys.argv[2]
m = onnx.load(src)
cnt = 0
for init in m.graph.initializer:
    if init.data_type == onnx.TensorProto.FLOAT16:
        arr = numpy_helper.to_array(init).astype("float32")
        init.CopyFrom(numpy_helper.from_array(arr, init.name))
        cnt += 1
for n in m.graph.node:
    if n.op_type == "Cast":
        for a in n.attribute:
            if a.name == "to" and a.i == 10:   # FLOAT16 -> FLOAT
                a.i = 1
def fix(t):
    if t.tensor_type.elem_type == 10:
        t.tensor_type.elem_type = 1
for vi in m.graph.value_info: fix(vi.type)
for t in list(m.graph.input) + list(m.graph.output): fix(t.type)
try:
    m = version_converter.convert_version(m, 19)
except Exception:
    pass
m.ir_version = 9
onnx.save(m, dst)
print("converted %d fp16 tensors -> %s" % (cnt, dst))
