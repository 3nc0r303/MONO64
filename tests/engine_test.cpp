#include "../Source/SynthEngine.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

static double rms(const std::vector<float>& x,int start=0) {
    double power=0;
    for(int i=start;i<static_cast<int>(x.size());++i)power+=double(x[i])*x[i];
    return std::sqrt(power/std::max(1,int(x.size())-start));
}
static double hpRms(double inputHz,double cutoff,bool steep) {
    constexpr int SR=48000,N=48000;
    mono64::HpCascade hp;
    hp.configure(cutoff,SR);
    double sum=0;
    for(int i=0;i<N;++i) {
        const double x=std::sin(2*mono64::PI*inputHz*i/SR);
        const double y=hp.process(x,steep);
        if(i>=SR/2)sum+=y*y;
    }
    return std::sqrt(sum/(N-SR/2));
}
int main() {
    constexpr int SR=48000,N=12000;
    mono64::SynthEngine s;
    s.prepare(SR);
    mono64::Parameters p;
    p.unison=64;p.detuneCents=24;p.driftDepthCents=7;
    p.driftRateHz=.5f;p.stereoSpread=100;
    s.setParameters(p);
    std::vector<float> l(N,0),r(N,0),ul(N),ur(N),sub(N);
    s.noteOn(48,1);
    s.render(l.data(),r.data(),N,ul.data(),ur.data(),sub.data());
    double energy=0,sideEnergy=0;
    for(int i=0;i<N;++i) {
        assert(std::isfinite(l[i])&&std::isfinite(r[i]));
        assert(std::abs(l[i])<=1.001&&std::abs(r[i])<=1.001);
        assert(std::isfinite(ul[i])&&std::isfinite(ur[i])&&std::isfinite(sub[i]));
        energy+=l[i]*l[i]+r[i]*r[i];
        sideEnergy+=std::abs(l[i]-r[i]);
    }
    assert(energy>0.001&&sideEnergy>0.001);
    s.noteOn(60,.8f);s.noteOff(60); // returns to held 48
    s.render(l.data(),r.data(),N);
    s.noteOff(48);
    for(int j=0;j<4;++j)s.render(l.data(),r.data(),N);
    s.allSoundOff();

    // A dual (24 dB) high-pass cuts 60 Hz but passes 3 kHz at 500 Hz cutoff.
    const double bass12=hpRms(60,500,false),bass24=hpRms(60,500,true);
    const double treble=hpRms(3000,500,true);
    assert(bass24 < bass12*.2);
    assert(bass24 < treble*.02);
    assert(treble>.5);

    // SUB remains exactly mono and is completely independent of both HPs.
    mono64::Parameters subOnly;
    for(auto& osc:subOnly.oscillators)osc.level=0;
    subOnly.unison=64;subOnly.subLevel=.8f;
    subOnly.unisonHpHz=9000;subOnly.sideHpHz=9000;
    subOnly.unisonHpOn=true;subOnly.sideHpOn=true;
    mono64::SynthEngine engineSub;
    engineSub.prepare(SR);engineSub.setParameters(subOnly);engineSub.noteOn(45,1);
    std::fill(l.begin(),l.end(),0);std::fill(r.begin(),r.end(),0);
    engineSub.render(l.data(),r.data(),N,ul.data(),ur.data(),sub.data());
    assert(rms(sub,SR/10)>.01);
    for(int i=0;i<N;++i) {
        assert(std::abs(ul[i])<1e-6&&std::abs(ur[i])<1e-6);
        assert(std::abs(l[i]-r[i])<1e-6);
    }

    // Side-only filter must not change the MID channel, but must attenuate SIDE.
    mono64::Parameters wide;
    wide.unison=16;wide.stereoSpread=100;wide.detuneCents=10;
    wide.driftDepthCents=0;wide.subLevel=0;
    wide.unisonHpOn=false;wide.sideHpHz=3000;
    wide.sideHpOn=false;
    mono64::SynthEngine noSide,withSide;
    noSide.prepare(SR);withSide.prepare(SR);
    noSide.setParameters(wide);wide.sideHpOn=true;withSide.setParameters(wide);
    noSide.noteOn(40,1);withSide.noteOn(40,1);
    std::vector<float> a(N),b(N),c(N),d(N);
    noSide.render(a.data(),b.data(),N);
    withSide.render(c.data(),d.data(),N);
    double midError=0,sideBefore=0,sideAfter=0;
    noSide.allSoundOff();withSide.allSoundOff();
    noSide.noteOn(40,1);withSide.noteOn(40,1);
    std::vector<float> rawL(N),rawR(N),filteredL(N),filteredR(N),unused(N);
    noSide.render(a.data(),b.data(),N,rawL.data(),rawR.data(),unused.data());
    withSide.render(c.data(),d.data(),N,filteredL.data(),filteredR.data(),unused.data());
    for(int i=SR/12;i<N;++i){
        midError+=std::abs((rawL[i]+rawR[i])-(filteredL[i]+filteredR[i]));
        sideBefore+=std::abs(rawL[i]-rawR[i]);
        sideAfter+=std::abs(filteredL[i]-filteredR[i]);
    }
    assert(midError<.005);
    assert(sideBefore>0.1);
    assert(sideAfter<sideBefore*.3);

    // At neutral 50%, the generalized saw and pulse exactly match v0.2.
    for (double phase : {0.0,.013,.23,.49,.52,.78,.97}) {
        const double blep0=mono64::polyBlep(phase,.002);
        assert(std::abs(mono64::mainWave(0,phase,.002,.5)-(2*phase-1-blep0))<1e-12);
        double shifted=std::fmod(phase+.5,1.);
        const double originalSquare=(phase<.5?1.:-1.)+blep0-mono64::polyBlep(shifted,.002);
        assert(std::abs(mono64::mainWave(1,phase,.002,.5)-originalSquare)<1e-12);
    }
    // Both saw and square respond to the single global knob. Square DC correction
    // and saw ramp asymmetry correction keep their average near zero.
    for(int wave=0;wave<2;++wave) {
        double difference=0,mean=.0;
        for(int k=0;k<4096;++k) {
            const double phase=(k+.5)/4096.;
            const double classic=mono64::mainWave(wave,phase,1./4096.,.5);
            const double shaped=mono64::mainWave(wave,phase,1./4096.,.23);
            difference+=std::abs(shaped-classic);
            mean+=shaped;
        }
        assert(difference/4096.>.1);
        assert(std::abs(mean/4096.)<.005);
    }
    // DSP changes audible signal with PWM LFO. And even when PWM is modulated,
    // the SUB tap remains bit-identical: shape never touches the sub oscillator.
    mono64::Parameters pwmSet;
    pwmSet.unison=8;pwmSet.detuneCents=0;pwmSet.driftDepthCents=0;
    pwmSet.stereoSpread=0;pwmSet.pulseWidth=.5;pwmSet.pwmRateHz=5;
    pwmSet.pwmDepth=0;pwmSet.subLevel=.35f;
    for(auto& osc:pwmSet.oscillators)osc.level=0;
    pwmSet.oscillators[0].level=.7f;
    pwmSet.oscillators[1].level=.4f;
    pwmSet.unisonHpOn=false;pwmSet.sideHpOn=false;
    mono64::SynthEngine fixedPw, movingPw;
    fixedPw.prepare(SR);movingPw.prepare(SR);
    fixedPw.setParameters(pwmSet);
    pwmSet.pwmDepth=.75f;movingPw.setParameters(pwmSet);
    fixedPw.noteOn(45,1);movingPw.noteOn(45,1);
    std::vector<float> x(N),y(N),z(N),q(N),su1(N),su2(N);
    fixedPw.render(x.data(),y.data(),N,nullptr,nullptr,su1.data());
    movingPw.render(z.data(),q.data(),N,nullptr,nullptr,su2.data());
    double change=0;
    for(int i=0;i<N;++i){change+=std::abs(x[i]-z[i]);assert(su1[i]==su2[i]);}
    assert(change/N>.002);
    // Existing LFOs can also target the very same global shape parameter.
    mono64::SynthEngine lfoPw;
    pwmSet.pwmDepth=0;
    pwmSet.lfos[0]={4.f,.8f,0,3};
    lfoPw.prepare(SR);lfoPw.setParameters(pwmSet);lfoPw.noteOn(45,1);
    std::fill(z.begin(),z.end(),0);std::fill(q.begin(),q.end(),0);
    lfoPw.render(z.data(),q.data(),N);
    double lfoChange=0;
    for(int i=0;i<N;++i)lfoChange+=std::abs(x[i]-z[i]);
    assert(lfoChange/N>.002);

    std::cout<<"PASS: 64 unison, mono note stack, sub isolation, 12/24dB HP, Mid/Side HP, finite output, common PWM Saw/Square + LFO\n";
}
