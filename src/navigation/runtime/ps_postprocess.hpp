#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <vector>

// Portable C++11 ARM PS reference. Raw arrays are contiguous CHW signed INT8.
// Call with the five distinct output scales from graph.json; no NMS is used.
namespace lds_net {
struct Detection { float x1,y1,x2,y2,score; int category; int anchor; };
struct Raw {
    const std::int8_t *p4_box, *p4_class, *p5_box, *p5_class, *road;
    std::array<float,5> scale;
};
struct Letterbox { int top,left,height,width; float ratio; };

inline float sigmoid(float x) {
    if (x>=0.f) return 1.f/(1.f+std::exp(-x));
    const float e=std::exp(x); return e/(1.f+e);
}

inline std::vector<Detection> detect(const Raw& raw, int max_det=300) {
    const int count=500, classes=10, k=std::min(max_det,count);
    if (k<=0) return {};
    std::array<std::array<float,10>,500> score;
    std::array<std::array<float,4>,500> boxes;
    std::array<float,500> best;
    for (int a=0;a<count;++a) {
        const bool p4=a<400;
        const int size=p4?20:10, stride=p4?16:32, index=p4?a:a-400;
        const int area=size*size;
        const auto box=p4?raw.p4_box:raw.p5_box;
        const auto cls=p4?raw.p4_class:raw.p5_class;
        const float bs=raw.scale[p4?0:2], cs=raw.scale[p4?1:3];
        const float cx=float(index%size)+.5f, cy=float(index/size)+.5f;
        boxes[a]={{(cx-float(box[index])*bs)*stride,
                   (cy-float(box[area+index])*bs)*stride,
                   (cx+float(box[2*area+index])*bs)*stride,
                   (cy+float(box[3*area+index])*bs)*stride}};
        best[a]=0.f;
        for (int c=0;c<classes;++c) {
            score[a][c]=sigmoid(float(cls[c*area+index])*cs);
            best[a]=std::max(best[a],score[a][c]);
        }
    }
    std::vector<int> anchors(count);
    std::iota(anchors.begin(),anchors.end(),0);
    std::stable_sort(anchors.begin(),anchors.end(),[&](int a,int b){return best[a]>best[b];});
    anchors.resize(k);
    std::vector<int> flattened(k*classes);
    std::iota(flattened.begin(),flattened.end(),0);
    std::stable_sort(flattened.begin(),flattened.end(),[&](int a,int b){
        return score[anchors[a/classes]][a%classes]>score[anchors[b/classes]][b%classes];
    });
    std::vector<Detection> result;
    result.reserve(k);
    for (int i=0;i<k;++i) {
        const int index=flattened[i], a=anchors[index/classes], c=index%classes;
        const auto &b=boxes[a];
        result.push_back({b[0],b[1],b[2],b[3],score[a][c],c,a});
    }
    return result;
}

inline void restore_boxes(std::vector<Detection>& boxes, const Letterbox& g, int h, int w) {
    for (auto &b:boxes) {
        b.x1=std::max(0.f,std::min(float(w),(b.x1-g.left)/g.ratio));
        b.x2=std::max(0.f,std::min(float(w),(b.x2-g.left)/g.ratio));
        b.y1=std::max(0.f,std::min(float(h),(b.y1-g.top)/g.ratio));
        b.y2=std::max(0.f,std::min(float(h),(b.y2-g.top)/g.ratio));
    }
}

inline float road320(const Raw& r, int c, int y, int x) {
    const float sy=std::max(0.f,(y+.5f)*.25f-.5f), sx=std::max(0.f,(x+.5f)*.25f-.5f);
    const int y0=std::min(int(sy),79), x0=std::min(int(sx),79);
    const int y1=std::min(y0+1,79), x1=std::min(x0+1,79);
    const float fy=sy-y0, fx=sx-x0;
    const auto p=r.road+c*6400;
    const float a=float(p[y0*80+x0])*(1.f-fx)+float(p[y0*80+x1])*fx;
    const float b=float(p[y1*80+x0])*(1.f-fx)+float(p[y1*80+x1])*fx;
    return (a*(1.f-fy)+b*fy)*r.scale[4];
}

inline std::vector<std::uint8_t> road(const Raw& r, int h, int w, const Letterbox& g) {
    if(h<=0 || w<=0 || g.height<=0 || g.width<=0) throw std::invalid_argument("Invalid image geometry");
    std::vector<std::uint8_t> labels(std::size_t(h)*w);
    // Evaluate the same two bilinear interpolations without materializing three
    // full-resolution float maps. Only final byte labels occupy image-size RAM.
    auto numerator320=[&](int c,int y,int x)->std::int64_t {
        const int sy=std::max(0,2*y-3),sx=std::max(0,2*x-3);
        const int y0=std::min(sy/8,79),x0=std::min(sx/8,79);
        const int y1=std::min(y0+1,79),x1=std::min(x0+1,79);
        const int fy=sy-y0*8,fx=sx-x0*8;
        const auto p=r.road+c*6400;
        return (std::int64_t(p[y0*80+x0])*(8-fx)+std::int64_t(p[y0*80+x1])*fx)*(8-fy)
              +(std::int64_t(p[y1*80+x0])*(8-fx)+std::int64_t(p[y1*80+x1])*fx)*fy;
    };
    for (int y=0;y<h;++y) for (int x=0;x<w;++x) {
        const std::int64_t sy=std::max(std::int64_t(0),(std::int64_t(2)*y+1)*g.height-h);
        const std::int64_t sx=std::max(std::int64_t(0),(std::int64_t(2)*x+1)*g.width-w);
        const int y0=std::min(int(sy/(2*h)),g.height-1),x0=std::min(int(sx/(2*w)),g.width-1);
        const int y1=std::min(y0+1,g.height-1), x1=std::min(x0+1,g.width-1);
        const std::int64_t fy=sy-std::int64_t(y0)*2*h,fx=sx-std::int64_t(x0)*2*w;
        std::int64_t best=INT64_MIN; int category=0;
        for (int c=0;c<3;++c) {
            const auto a=numerator320(c,y0+g.top,x0+g.left)*(2*w-fx)+numerator320(c,y0+g.top,x1+g.left)*fx;
            const auto b=numerator320(c,y1+g.top,x0+g.left)*(2*w-fx)+numerator320(c,y1+g.top,x1+g.left)*fx;
            const auto value=a*(2*h-fy)+b*fy;
            if(value>best) {best=value;category=c;}
        }
        labels[std::size_t(y)*w+x]=std::uint8_t(category);
    }
    return labels;
}
} // namespace lds_net
