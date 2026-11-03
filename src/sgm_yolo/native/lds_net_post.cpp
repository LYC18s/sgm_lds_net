// C++11 board postprocessing. Only 7000 detection bytes cross into the CPU.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numeric>
#ifdef _WIN32
#define RL_EXPORT extern "C" __declspec(dllexport)
#else
#define RL_EXPORT extern "C" __attribute__((visibility("default")))
#endif

struct LDSNetDetection {
    float x1,y1,x2,y2,score;
    std::int32_t category,anchor;
};

RL_EXPORT std::uint32_t lds_net_post_abi() {return 0x524c0001U;}

// Fixed-size stable merge sort: no heap allocation or C++ exception runtime.
// This also keeps the ARM library independent of a board's libstdc++ version.
template<std::size_t N,class Compare>
static void stable_indices(std::array<int,N>& values,Compare before) noexcept {
    std::array<int,N> temporary;
    for(std::size_t width=1;width<N;width*=2) {
        for(std::size_t start=0;start<N;start+=width*2) {
            const std::size_t middle=std::min(start+width,N),end=std::min(start+2*width,N);
            std::size_t left=start,right=middle;
            for(std::size_t out=start;out<end;++out) {
                if(left<middle && (right>=end || !before(values[right],values[left]))) temporary[out]=values[left++];
                else temporary[out]=values[right++];
            }
        }
        values=temporary;
    }
}

RL_EXPORT int lds_net_detect(const std::int8_t* data, std::size_t bytes,
                             const float* scales,float threshold,
                             LDSNetDetection* output,int capacity) noexcept {
    if(!data || !scales || !output || bytes!=7000 || capacity<0 ||
       capacity>300 || !std::isfinite(threshold) || threshold<0.f || threshold>1.f) return -1;
    for(int i=0;i<4;++i) if(!std::isfinite(scales[i]) || scales[i]<=0.f) return -1;
    {
        std::array<std::array<float,10>,500> scores;
        std::array<std::array<float,4>,500> boxes;
        std::array<float,500> best;
        static thread_local std::array<std::array<float,256>,2> sigmoid;
        static thread_local std::array<float,2> sigmoid_scales={{0.f,0.f}};
        for(int head=0;head<2;++head) if(sigmoid_scales[head]!=scales[head*2+1]) {
            for(int code=0;code<256;++code) {
                const int signed_code=code<128 ? code : code-256;
                const float x=float(signed_code)*scales[head*2+1];
                if(x>=0.f) sigmoid[head][code]=1.f/(1.f+std::exp(-x));
                else {const float e=std::exp(x);sigmoid[head][code]=e/(1.f+e);}
            }
            sigmoid_scales[head]=scales[head*2+1];
        }
        for(int a=0;a<500;++a) {
            const int head=a<400?0:1,side=head?10:20,count=side*side;
            const int index=head?a-400:a,stride=head?32:16;
            const auto regression=data+(head?5600:0);
            const auto classification=data+(head?6000:1600);
            const float cx=float(index%side)+.5f,cy=float(index/side)+.5f,s=scales[head*2];
            boxes[a]={{(cx-float(regression[index])*s)*stride,
                       (cy-float(regression[count+index])*s)*stride,
                       (cx+float(regression[2*count+index])*s)*stride,
                       (cy+float(regression[3*count+index])*s)*stride}};
            best[a]=0.f;
            for(int c=0;c<10;++c) {
                scores[a][c]=sigmoid[head][std::uint8_t(classification[c*count+index])];
                best[a]=std::max(best[a],scores[a][c]);
            }
        }
        // Both stages always use k=300 before confidence filtering. Changing k
        // to output capacity would change which anchors/classes are eligible.
        std::array<int,500> anchors;
        std::iota(anchors.begin(),anchors.end(),0);
        stable_indices(anchors,[&](int a,int b){return best[a]>best[b];});
        std::array<int,3000> flattened;
        std::iota(flattened.begin(),flattened.end(),0);
        stable_indices(flattened,[&](int a,int b){
            return scores[anchors[a/10]][a%10]>scores[anchors[b/10]][b%10];
        });
        int written=0;
        for(int i=0;i<300 && written<capacity;++i) {
            const int a=anchors[flattened[i]/10],c=flattened[i]%10;
            const float score=scores[a][c];
            if(score<threshold) continue;
            const auto &b=boxes[a];
            const auto clip=[](float v){return std::max(0.f,std::min(480.f,v*1.5f));};
            output[written++]={clip(b[0])+80.f,clip(b[1]),clip(b[2])+80.f,clip(b[3]),score,c,a};
        }
        return written;
    }
}
