// X11 全屏截图工具（无 ImageMagick 时用）
// g++ scripts/xshot.cpp -o /tmp/xshot -lX11 $(pkg-config --cflags --libs opencv4)
#include <opencv2/opencv.hpp>
#include <X11/Xlib.h>
#include <X11/Xutil.h>

int main(int argc, char** argv) {
    Display* d = XOpenDisplay(nullptr);
    if (!d) return 1;
    Window root = DefaultRootWindow(d);
    XWindowAttributes wa;
    XGetWindowAttributes(d, root, &wa);
    XImage* im = XGetImage(d, root, 0, 0, wa.width, wa.height, AllPlanes, ZPixmap);
    if (!im) return 2;
    cv::Mat out(wa.height, wa.width, CV_8UC3);
    for (int y = 0; y < wa.height; ++y)
        for (int x = 0; x < wa.width; ++x) {
            unsigned long p = XGetPixel(im, x, y);
            out.at<cv::Vec3b>(y, x) = cv::Vec3b(p & 0xff, (p >> 8) & 0xff, (p >> 16) & 0xff);
        }
    cv::imwrite(argc > 1 ? argv[1] : "xshot.png", out);
    XDestroyImage(im);
    XCloseDisplay(d);
    return 0;
}
