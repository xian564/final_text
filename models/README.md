# 数字模板（可选）

放 5 张灰度数字图，文件名固定：

```
models/digit_1.png
models/digit_2.png
models/digit_3.png
digit_4.png
digit_5.png
```

要求：
- 数字为**白字黑底**（脚本会自动 Otsu + 反色，所以黑底白字反过来也行）；
- 尽量正对、无透视；
- 大小不限，加载时统一缩放到 20×30。

没有模板时，传统链路 `number=0`（未知），不影响检测。
