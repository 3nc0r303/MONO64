#pragma once
// MONO64 v0.3 DSP: portable C++17, no JUCE dependencies.
// One played pitch, up to 64 independent unison voices per oscillator.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace mono64 {
constexpr double PI = 3.1415926535897932384626433832795;
constexpr int MAX_UNISON = 64;

struct OscParams {
    int waveform = 0;  // 0 = saw (bandlimited PolyBLEP), 1 = square (PolyBLEP)
    int octave = 0;
    float level = 0;
};
struct EnvParams { float attack = .01f, decay = .2f, sustain = .7f, release = .3f; };
struct LfoParams {
    float rate = 2, depth = 0;
    int shape = 0;       // modulation only: sine, triangle, sample-and-hold
    int destination = 0; // pitch, cutoff, amplitude, global PWM/shape
};
struct Parameters {
    std::array<OscParams, 3> oscillators {{ { 0, 0, .75f }, { 1, 0, .3f }, { 0, -1, 0 } }};
    int unison = 8;
    float detuneCents = 18, stereoSpread = 75, driftDepthCents = 4;
    float driftRateHz = .2f, ampVariation = 15, glideMs = 45;
    // Global shape for ALL three unison oscillators. 0.5 is classic saw / 50% square.
    // Saw uses phase-warped asymmetric ramp; square uses true variable pulse duty.
    float pulseWidth = .5f;  // normalized breakpoint / pulse duty, clamped .05.. .95
    float pwmRateHz = .3f;  // dedicated free-running modulation
    float pwmDepth = 0;     // fraction 0..1 -> +/-45 percentage points

    bool legato = true;
    float subLevel = .35f; // ONE sine oscillator, independent of unison; never sent to HPs
    int subOctave = -1;    // -1 / -2 below played note
    float cutoffHz = 4300, resonance = .25f, filterEnvOctaves = .9f;
    float drive = .12f;
    bool unisonHpOn = true, sideHpOn = true;
    float unisonHpHz = 100, sideHpHz = 180;
    bool unisonHp24 = true, sideHp24 = true;
    EnvParams ampEnv { .008f, .18f, .72f, .35f };
    EnvParams filterEnv { .002f, .23f, 0, .27f };
    std::array<LfoParams, 2> lfos {{{4.5f,0,0,0},{.33f,0,0,1}}};
    float master = .55f;
};
inline double clampD(double v,double lo,double hi) { return std::max(lo,std::min(v,hi)); }
inline double centsRatio(double c) { return std::exp2(c/1200.); }
inline double midiHz(double n) { return 440.*std::exp2((n-69.)/12.); }
inline double polyBlep(double p,double dp) {
    if (dp<=0) return 0;
    if (p<dp) { double t=p/dp; return t+t-t*t-1; }
    if (p>1-dp) { double t=(p-1)/dp; return t*t+t+t+1; }
    return 0;
}
// Both audible waveform types respond to ONE shared pulse-width/shape parameter.
// Saw: asymmetric phase-distorted ramp. Its slope changes at 'width', but the
// end-of-cycle jump remains at phase 0. At width=.5 this is the classic saw.
// Pulse: variable-width square, with band-limited rising/falling edges.
// Sub sine intentionally does not pass through this function.
inline double mainWave(int shape,double p,double dp,double width=.5) {
    const double w=clampD(width,.05,.95);
    if (shape==0) {
        const double warped = p<w ? .5*p/w : .5+.5*(p-w)/(1-w);
        // Remove DC: asymmetric ramp's mean is (.5 - w).
        return 2*warped-1-polyBlep(p,dp)-(.5-w);
    }
    double falling=p-w;
    if(falling<0)falling+=1;
    // Remove DC: rectangular pulse duty w has mean (2*w-1).
    return (p<w?1.:-1.)+polyBlep(p,dp)-polyBlep(falling,dp)-(2*w-1);
}
class Envelope {
public:
    enum class Stage { idle, attack, decay, sustain, release };
    void setSampleRate(double sr) { sampleRate = sr>1?sr:44100; }
    void setParams(EnvParams p) { params=p; }
    bool isActive() const { return stage!=Stage::idle; }
    void reset() { stage=Stage::idle; level=0; elapsed=0; }
    void noteOn(bool retrigger) {
        if (!retrigger && isActive()) return;
        stage=Stage::attack; attackStart=level; elapsed=0;
    }
    void noteOff() {
        if (!isActive()||stage==Stage::release) return;
        stage=Stage::release; releaseStart=level; elapsed=0;
    }
    double next() {
        ++elapsed;
        const double sustain=clampD(params.sustain,0,1);
        switch(stage) {
            case Stage::idle: return 0;
            case Stage::attack: {
                double t=clampD(elapsed/(std::max(.001f,params.attack)*sampleRate),0,1);
                level=attackStart+(1-attackStart)*t;
                if(t>=1) { stage=Stage::decay; elapsed=0; }
                break;
            }
            case Stage::decay: {
                double t=clampD(elapsed/(std::max(.001f,params.decay)*sampleRate),0,1);
                level=1+(sustain-1)*t;
                if(t>=1) { stage=Stage::sustain; level=sustain; }
                break;
            }
            case Stage::sustain: level=sustain; break;
            case Stage::release: {
                double t=clampD(elapsed/(std::max(.001f,params.release)*sampleRate),0,1);
                level=releaseStart*(1-t);
                if(t>=1) reset();
                break;
            }
        }
        return level;
    }
private:
    double sampleRate=44100,level=0,attackStart=0,releaseStart=0,elapsed=0;
    Stage stage=Stage::idle;
    EnvParams params;
};
// TPT 2-pole state-variable filter. Cascading two produces 24 dB/octave.
class Svf {
public:
    void reset() { ic1=ic2=0; }
    void configure(double freq,double q,double sr) {
        double g=std::tan(PI*clampD(freq,15.,sr*.44)/sr);
        double k=1./clampD(q,.5,14.);
        a1=1./(1+g*(g+k)); a2=g*a1; a3=g*a2;
    }
    void process(double x,double& low,double& high) {
        const double v3=x-ic2;
        const double v1=a1*ic1+a2*v3;
        const double v2=ic2+a2*ic1+a3*v3;
        ic1=2*v1-ic1; ic2=2*v2-ic2;
        low=v2; high=x-(1./qCache)*v1-v2; // qCache is configured together with coefficients
    }
    void configureButterworth(double freq,double sr) { qCache=std::sqrt(.5); configure(freq,qCache,sr); }
    void configureResonant(double freq,double q,double sr) { qCache=q; configure(freq,q,sr); }
private:
    double ic1=0,ic2=0,a1=1,a2=0,a3=0,qCache=std::sqrt(.5);
};
class HpCascade {
public:
    void reset() { a.reset();b.reset(); }
    void configure(double hz,double sr) { a.configureButterworth(hz,sr); b.configureButterworth(hz,sr); }
    double process(double x,bool slope24) {
        double lo=0,hi=0;
        a.process(x,lo,hi);
        const double first=hi;
        b.process(first,lo,hi); // keep both stages warm when switching slopes
        return slope24?hi:first;
    }
private:
    Svf a,b;
};

class SynthEngine {
public:
    SynthEngine() { initVoices(); }
    void prepare(double sr) {
        sampleRate=sr>1?sr:44100;
        ampEnv.setSampleRate(sampleRate); filterEnv.setSampleRate(sampleRate);
        reset();
    }
    void reset() {
        heldCount=0; playingNote=-1; pitchBendSemitones=0;
        currentPitch=targetPitch=60; velocity=0; subPhase=0; pwmPhase=0; smoothPw=.5;
        ampEnv.reset();filterEnv.reset();
        for(auto& f:lowpass)f.reset();
        for (auto& f : unisonHp) f.reset();
        sideHp.reset();
        for(auto& l:lfos){l.phase=0;l.hold=0;}
        counter=0; smoothMaster=.5; initVoices(); updateControl();
    }
    void setParameters(const Parameters& p) { params=p; ampEnv.setParams(p.ampEnv);filterEnv.setParams(p.filterEnv); }
    void noteOn(int note,float vel) {
        if(note<0||note>127)return;
        removeHeld(note);
        if(heldCount<128)held[heldCount++]=note;
        bool wasHeld=playingNote>=0&&ampEnv.isActive();
        playingNote=note; targetPitch=note;
        if(!wasHeld||params.glideMs<.1f)currentPitch=targetPitch;
        velocity=clampD(vel,0,1);
        const bool retrigger=!params.legato||!wasHeld;
        ampEnv.noteOn(retrigger);filterEnv.noteOn(retrigger);
        if(retrigger){initVoices();subPhase=0;}
    }
    void noteOff(int note) {
        if(note<0||note>127)return;
        bool current=note==playingNote;
        removeHeld(note);
        if(!current)return;
        if(heldCount>0) {
            playingNote=held[heldCount-1];targetPitch=playingNote;
            if(params.glideMs<.1f)currentPitch=targetPitch;
            if(!params.legato){ampEnv.noteOn(true);filterEnv.noteOn(true);}
        } else {playingNote=-1;ampEnv.noteOff();filterEnv.noteOff();}
    }
    void allNotesOff(){heldCount=0;playingNote=-1;ampEnv.noteOff();filterEnv.noteOff();}
    void allSoundOff(){reset();}
    void pitchWheel(int val){pitchBendSemitones=clampD((val-8192)/8192.,-1,1)*2;}
    // Optional capture arrays contain separated unison L/R and SUB mono for the analyzer.
    // All taps are AFTER the HPs, before summing the buses. No heap allocation.
    void render(float* left,float* right,int n,float* tapUniL=nullptr,float* tapUniR=nullptr,float* tapSub=nullptr) {
        for(int i=0;i<n;++i) {
            double ul=0,ur=0,sub=0;
            double env=ampEnv.next(),filtEnv=filterEnv.next();
            if((counter++&31)==0)updateControl();
            if(env>0) {
                const int count=std::clamp(params.unison,1,MAX_UNISON);
                double glideTime=params.glideMs/1000.;
                if(glideTime<=.0001)currentPitch=targetPitch;
                else currentPitch+=(targetPitch-currentPitch)*(1-std::exp(-1/(sampleRate*glideTime)));
                double pitchCents=0,cutoffOct=0,ampMod=1,additionalPw=0;
                for(int idx=0;idx<2;++idx) {
                    double val=nextLfo(idx);
                    const auto& lfo=params.lfos[idx];
                    if(lfo.destination==0)pitchCents+=val*lfo.depth*100.;
                    else if(lfo.destination==1)cutoffOct+=val*lfo.depth*3.;
                    else if(lfo.destination==2)ampMod*=1-lfo.depth*.5*(val+1);
                    else if(lfo.destination==3)additionalPw+=val*lfo.depth*.45;
                }
                // Dedicated global PWM oscillator modulates both saw and pulse.
                // LFO 1/2 can additionally target this SAME global shape control.
                const double pwmLfo=std::sin(2*PI*pwmPhase);
                pwmPhase+=clampD(params.pwmRateHz,.02,20.)/sampleRate;
                pwmPhase-=std::floor(pwmPhase);
                const double targetPw=clampD(params.pulseWidth +
                    pwmLfo*clampD(params.pwmDepth,0,1)*.45+additionalPw,.05,.95);
                // 5ms smoothing eliminates parameter-automation zippering.
                smoothPw+=(targetPw-smoothPw)*(1-std::exp(-1./(.005*sampleRate)));
                const double baseHz=midiHz(currentPitch+pitchBendSemitones+pitchCents/100.);
                const double norm=.30/std::sqrt(static_cast<double>(count));
                for(int v=0;v<count;++v) {
                    double voice=0;
                    for(int o=0;o<3;++o) {
                        const auto& osc=params.oscillators[o];
                        if(osc.level<=.00001f)continue;
                        const double inc=std::min(.49,baseHz*oscRatio[o]*ratios[v]/sampleRate);
                        voice+=mainWave(osc.waveform,phases[o][v],inc,smoothPw)*osc.level;
                        phases[o][v]+=inc;
                        if(phases[o][v]>=1)phases[o][v]-=1;
                    }
                    const double a=voice*norm*ampVar[v];
                    ul+=a*panL[v];ur+=a*panR[v];
                }
                // Exactly ONE centered sine sub oscillator regardless of unison count/spread.
                const double subInc=std::min(.49,baseHz*std::exp2(params.subOctave)/sampleRate);
                sub=std::sin(2*PI*subPhase)*params.subLevel*.32;
                subPhase+=subInc;
                if(subPhase>=1)subPhase-=1;
                const double amplitude=env*velocity*ampMod;
                ul*=amplitude;ur*=amplitude;sub*=amplitude;
                if((counter&31)==1) {
                    double freq=params.cutoffHz*std::exp2(params.filterEnvOctaves*filtEnv+cutoffOct);
                    double q=.707+std::pow(params.resonance,2.)*11.;
                    for(auto& f:lowpass)f.configureResonant(freq,q,sampleRate);
                }
            }
            double lo,hi;
            lowpass[0].process(ul,lo,hi); ul=lo;
            lowpass[1].process(ur,lo,hi); ur=lo;
            const double drive=1+params.drive*9.;
            ul=std::tanh(ul*drive)/std::tanh(drive);
            ur=std::tanh(ur*drive)/std::tanh(drive);
            // Dual high-pass module, unison bus ONLY (sub is not in either path).
            const double ulHp=unisonHp[0].process(ul,params.unisonHp24);
            const double urHp=unisonHp[1].process(ur,params.unisonHp24);
            if(params.unisonHpOn){ul=ulHp;ur=urHp;}
            const double mid=.5*(ul+ur),side=.5*(ul-ur);
            const double sideFiltered=sideHp.process(side,params.sideHp24);
            const double adjustedSide=params.sideHpOn?sideFiltered:side;
            ul=mid+adjustedSide;ur=mid-adjustedSide;
            const double gain=clampD(params.master,0,1);
            smoothMaster+=(gain-smoothMaster)*.002;
            ul*=smoothMaster;ur*=smoothMaster;sub*=smoothMaster;
            // Color-coded analyzer taps represent each component before final output sum.
            if(tapUniL)tapUniL[i]=static_cast<float>(ul);
            if(tapUniR)tapUniR[i]=static_cast<float>(ur);
            if(tapSub)tapSub[i]=static_cast<float>(sub);
            // Safety soft-clip; peak limiting is not true-peak limiting.
            left[i]+=static_cast<float>(std::tanh(ul+sub));
            right[i]+=static_cast<float>(std::tanh(ur+sub));
        }
    }
private:
    struct LfoState {double phase=0,hold=0;};
    double nextLfo(int idx){
        auto& s=lfos[idx];const auto& p=params.lfos[idx];
        double val=p.shape==0?std::sin(2*PI*s.phase):p.shape==1?1-4*std::abs(s.phase-.5):s.hold;
        s.phase+=clampD(p.rate,.005,80)/sampleRate;
        if(s.phase>=1){s.phase-=1;s.hold=2*random01()-1;}
        return val;
    }
    void removeHeld(int note){
        for(int i=0;i<heldCount;++i)if(held[i]==note){
            for(int j=i;j+1<heldCount;++j)held[j]=held[j+1];
            --heldCount;return;
        }
    }
    double random01(){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return static_cast<double>(rng)/4294967296.;}
    void initVoices(){
        for(int v=0;v<MAX_UNISON;++v){
            driftPhases[v]=std::fmod(v*.61803398875,1.);
            driftRates[v]=.66+.66*((v*43+17)%97)/96.;
            ampSeeds[v]=((v*73+9)%101)/50.-1.;
            for(int o=0;o<3;++o)phases[o][v]=v==0?0.:((v*47+o*23)%101)/101.;
        }
    }
    void updateControl(){
        const int n=std::clamp(params.unison,1,MAX_UNISON);
        double spread=clampD(params.stereoSpread/100.,0,1),detune=clampD(params.detuneCents,0,100);
        double av=clampD(params.ampVariation/100.,0,1);
        for(int o=0;o<3;++o)oscRatio[o]=std::exp2(clampD(params.oscillators[o].octave,-2,2));
        for(int v=0;v<n;++v){
            double pos=n==1?0:(2.*v/(n-1)-1);
            double drift=std::sin(2*PI*driftPhases[v])*params.driftDepthCents;
            ratios[v]=centsRatio(pos*detune+drift);
            driftPhases[v]+=params.driftRateHz*driftRates[v]*32/sampleRate;
            driftPhases[v]-=std::floor(driftPhases[v]);
            double pan=spread*pos;
            panL[v]=std::sqrt(.5*(1-pan));panR[v]=std::sqrt(.5*(1+pan));
            ampVar[v]=std::max(.1,1+av*ampSeeds[v]*.65);
        }
        // Both filters are continuously updated, even when bypassed: immediate switching.
        for(auto& hp:unisonHp)hp.configure(params.unisonHpHz,sampleRate);
        sideHp.configure(params.sideHpHz,sampleRate);
    }
    Parameters params;
    Envelope ampEnv,filterEnv;
    Svf lowpass[2];
    HpCascade unisonHp[2],sideHp;
    std::array<std::array<double,MAX_UNISON>,3> phases{};
    std::array<double,MAX_UNISON> ratios{},panL{},panR{},ampVar{};
    std::array<double,MAX_UNISON> driftPhases{},driftRates{},ampSeeds{};
    std::array<double,3> oscRatio{};
    std::array<LfoState,2> lfos{};
    std::array<int,128> held{};
    int heldCount=0,playingNote=-1,counter=0;
    double sampleRate=44100,targetPitch=60,currentPitch=60;
    double pitchBendSemitones=0,velocity=0,subPhase=0,smoothMaster=.5;
    double pwmPhase=0,smoothPw=.5;
    uint32_t rng=0xBADC0DEu;
};
} // namespace mono64
