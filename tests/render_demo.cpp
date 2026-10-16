#include "../Source/SynthEngine.h"
#include <vector>
#include <fstream>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <iostream>

static void writeInt16(std::ofstream& f, int16_t v) { f.write(reinterpret_cast<const char*>(&v), 2); }
static void writeInt32(std::ofstream& f, int32_t v) { f.write(reinterpret_cast<const char*>(&v), 4); }
int main(int argc, char** argv) {
    const char* filename = argc > 1 ? argv[1] : "tests/mono64_demo.wav";
    constexpr int SR=48000;
    constexpr int NS=SR*7;
    std::vector<float> l(NS,0),r(NS,0);
    mono64::SynthEngine s;
    mono64::Parameters p;
    p.unison=64; p.detuneCents=29.0f; p.stereoSpread=100;
    p.driftDepthCents=7; p.driftRateHz=0.18f;
    p.oscillators[0]={0,0,0.8f}; p.oscillators[1]={1,-1,0.3f};
    p.oscillators[2]={0,1,0.2f};
    p.cutoffHz=1200; p.resonance=0.32f; p.filterEnvOctaves=1.6f;
    p.ampEnv={0.25f,0.9f,0.84f,1.0f};
    p.filterEnv={0.15f,1.2f,0.25f,0.8f};
    p.lfos[0]={0.22f,0.25f,0,1};
    p.lfos[1]={4.0f,0.08f,0,0};
    p.subLevel=0.35f; p.drive=0.2f;
    p.pulseWidth=.35f; p.pwmRateHz=.47f; p.pwmDepth=.35f;
    p.unisonHpOn=true; p.unisonHpHz=135; p.unisonHp24=true;
    p.sideHpOn=true; p.sideHpHz=300; p.sideHp24=true;
    s.prepare(SR);
    s.setParameters(p);
    s.noteOn(36,0.85f);
    s.render(l.data(),r.data(),SR*2);
    s.noteOn(43,0.88f); // legato glide
    s.render(l.data()+SR*2,r.data()+SR*2,SR*2);
    s.noteOn(48,0.82f);
    s.render(l.data()+SR*4,r.data()+SR*4,SR);
    s.noteOff(48); s.noteOff(43); s.noteOff(36);
    s.render(l.data()+SR*5,r.data()+SR*5,SR*2);
    double peak = 0;
    for(int i=0;i<NS;++i) peak = std::max({peak,std::abs(double(l[i])),std::abs(double(r[i]))});
    const double gain = peak > 0.92 ? 0.92/peak : 1.0;
    std::ofstream f(filename,std::ios::binary);
    if (!f) return 1;
    f.write("RIFF",4); writeInt32(f,36+NS*4); f.write("WAVEfmt ",8);
    writeInt32(f,16); writeInt16(f,1); writeInt16(f,2);
    writeInt32(f,SR); writeInt32(f,SR*4); writeInt16(f,4); writeInt16(f,16);
    f.write("data",4); writeInt32(f,NS*4);
    for (int i=0;i<NS;++i) {
        auto convert = [&](float v) -> int16_t { return static_cast<int16_t>(std::lround(
            std::max(-1.0, std::min(1.0, double(v)*gain)) * 32767.0)); };
        writeInt16(f,convert(l[i])); writeInt16(f,convert(r[i]));
    }
    std::cout << "Saved " << filename << " peak=" << peak << "\n";
}
