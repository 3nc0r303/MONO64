#include <juce_audio_processors/juce_audio_processors.h>
#include <juce_audio_utils/juce_audio_utils.h>
#include <juce_dsp/juce_dsp.h>
#include "SynthEngine.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <vector>

namespace {
using Tree = juce::AudioProcessorValueTreeState;
using APF = juce::AudioParameterFloat;
using API = juce::AudioParameterInt;
using APC = juce::AudioParameterChoice;
using APB = juce::AudioParameterBool;

Tree::ParameterLayout createParams() {
    Tree::ParameterLayout layout;
    auto f = [&](const char* id,const juce::String& name,float min,float max,float def,
                 float skew=1.f,float step=.001f) {
        layout.add(std::make_unique<APF>(juce::ParameterID{id,1},name,
            juce::NormalisableRange<float>{min,max,step,skew},def));
    };
    auto c = [&](const char* id,const juce::String& name,const juce::StringArray& values,int def) {
        layout.add(std::make_unique<APC>(juce::ParameterID{id,1},name,values,def));
    };
    auto integer = [&](const char* id,const juce::String& name,int lo,int hi,int def) {
        layout.add(std::make_unique<API>(juce::ParameterID{id,1},name,lo,hi,def));
    };
    auto boolean = [&](const char* id,const juce::String& name,bool def) {
        layout.add(std::make_unique<APB>(juce::ParameterID{id,1},name,def));
    };
    const juce::StringArray waves{"Saw","Square"};
    for (int n=1;n<=3;++n) {
        auto prefix="osc"+juce::String(n);
        auto wave=prefix+"_wave",oct=prefix+"_oct",level=prefix+"_level";
        c(wave.toRawUTF8(),"Osc "+juce::String(n)+" Wave",waves,n==2?1:0);
        integer(oct.toRawUTF8(),"Osc "+juce::String(n)+" Octave",-2,2,n==3?-1:0);
        f(level.toRawUTF8(),"Osc "+juce::String(n)+" Level",0,1,n==1?.75f:n==2?.3f:0.f);
    }
    integer("unison","Unison Voices",1,64,8);
    f("detune","Static Detune (cents)",0,100,18,.7f);
    f("spread","Stereo Spread (%)",0,100,75);
    f("drift_depth","Drift Depth (cents)",0,40,4);
    f("drift_rate","Drift Frequency (Hz)",0,10,.2f,.45f);
    f("amp_variation","Voice Amp Variation (%)",0,100,15);
    f("glide","Glide (ms)",0,1000,45,.55f);
    f("master","Master Volume",0,1,.55f);
    boolean("legato","Legato",true);
    f("pw","Global PW / Saw Shape (%)",5,95,50);
    f("pwm_rate","PWM Mod Rate (Hz)",.02f,20,.3f,.38f);
    f("pwm_depth","PWM Mod Depth (%)",0,100,0);

    f("sub","Single-Voice Sub Level",0,1,.35f);
    c("sub_oct","Sub Octave",{"-1 octave","-2 octaves"},0);
    f("cutoff","Main Lowpass (Hz)",30,20000,4300,.28f);
    f("resonance","Lowpass Resonance",0,1,.25f);
    f("filter_amount","Filter Envelope (octaves)",-5,5,.9f);
    f("drive","Unison Saturation",0,1,.12f);

    boolean("hp_full_on","Unison HP Enabled",true);
    f("hp_full_cut","Unison HP Cutoff (Hz)",20,16000,100,.23f);
    boolean("hp_full_24","Unison HP 24dB",true);
    boolean("hp_side_on","Side HP Enabled",true);
    f("hp_side_cut","Side HP Cutoff (Hz)",20,16000,180,.23f);
    boolean("hp_side_24","Side HP 24dB",true);

    for (const char* env : {"amp","filter"}) {
        const bool isAmp = juce::String(env)=="amp";
        const juce::String prefix(env);
        const auto a=prefix+"_attack",d=prefix+"_decay",s=prefix+"_sustain",r=prefix+"_release";
        f(a.toRawUTF8(),"Attack (s)",.001f,8,isAmp?.008f:.002f,.28f);
        f(d.toRawUTF8(),"Decay (s)",.001f,8,isAmp?.18f:.23f,.35f);
        f(s.toRawUTF8(),"Sustain",0,1,isAmp?.72f:0.f);
        f(r.toRawUTF8(),"Release (s)",.001f,12,isAmp?.35f:.27f,.35f);
    }
    const juce::StringArray lfoWaves{"Sine","Triangle","S&H"};
    const juce::StringArray targets{"Pitch","Filter","Amplitude","PWM / Shape"};
    for (int n=1;n<=2;++n) {
        const auto prefix="lfo"+juce::String(n)+"_";
        const auto wave=prefix+"shape",dest=prefix+"dest",rate=prefix+"rate",depth=prefix+"depth";
        c(wave.toRawUTF8(),"LFO Shape",lfoWaves,0);
        c(dest.toRawUTF8(),"LFO Target",targets,n==1?0:1);
        f(rate.toRawUTF8(),"LFO Rate (Hz)",.01f,30,n==1?4.5f:.33f,.38f);
        f(depth.toRawUTF8(),"LFO Depth (%)",0,100,0);
    }
    return layout;
}

// SPSC FIFO: producer=audio thread, consumer=GUI message thread.
// No allocation, FFT, locks or painting on audio thread.
struct SpectrumFrame {float unisonL=0,unisonR=0,sub=0;};
class SpectrumPipe final {
public:
    static constexpr int SIZE=32768;
    SpectrumPipe():fifo(SIZE){}
    void prepare(double sr) {
        sampleRate.store(sr,std::memory_order_relaxed);
        fifo.reset(); // prepareToPlay must not be concurrent with processBlock
    }
    double getSampleRate()const{return sampleRate.load(std::memory_order_relaxed);}
    void push(const float* l,const float* r,const float* s,int samples) {
        int a,an,b,bn;
        fifo.prepareToWrite(samples,a,an,b,bn);
        for(int i=0;i<an;++i)ring[static_cast<size_t>(a+i)]={l[i],r[i],s[i]};
        for(int i=0;i<bn;++i)ring[static_cast<size_t>(b+i)]={l[an+i],r[an+i],s[an+i]};
        fifo.finishedWrite(an+bn);
    }
    int pop(SpectrumFrame* dest,int maximum) {
        int a,an,b,bn;
        fifo.prepareToRead(maximum,a,an,b,bn);
        for(int i=0;i<an;++i)dest[i]=ring[static_cast<size_t>(a+i)];
        for(int i=0;i<bn;++i)dest[an+i]=ring[static_cast<size_t>(b+i)];
        fifo.finishedRead(an+bn);
        return an+bn;
    }
private:
    juce::AbstractFifo fifo;
    std::array<SpectrumFrame,SIZE> ring{};
    std::atomic<double> sampleRate{44100};
};

class Mono64Processor final:public juce::AudioProcessor {
public:
    Mono64Processor():AudioProcessor(BusesProperties().withOutput("Output",juce::AudioChannelSet::stereo(),true)),
        parameters(*this,nullptr,"MONO64_STATE",createParams()){}
    const juce::String getName()const override{return "MONO64";}
    bool acceptsMidi()const override{return true;}
    bool producesMidi()const override{return false;}
    bool isMidiEffect()const override{return false;}
    double getTailLengthSeconds()const override{return 12.;}
    int getNumPrograms()override{return 1;}
    int getCurrentProgram()override{return 0;}
    void setCurrentProgram(int)override{}
    const juce::String getProgramName(int)override{return {};}
    void changeProgramName(int,const juce::String&)override{}
    bool isBusesLayoutSupported(const BusesLayout& b)const override{
        return b.getMainOutputChannelSet()==juce::AudioChannelSet::stereo();
    }
    void prepareToPlay(double sr,int)override{engine.prepare(sr);spectrum.prepare(sr);}
    void releaseResources()override{}
    void processBlock(juce::AudioBuffer<float>& audio,juce::MidiBuffer& midi)override {
        juce::ScopedNoDenormals noDenormals;
        const int samples=audio.getNumSamples();
        audio.clear();
        if(audio.getNumChannels()<2)return;
        keyboardState.processNextMidiBuffer(midi,0,samples,true);
        updateEngineParameters();
        auto* left=audio.getWritePointer(0);
        auto* right=audio.getWritePointer(1);
        int cursor=0;
        for(const auto metadata:midi) {
            const int boundary=juce::jlimit(cursor,samples,metadata.samplePosition);
            renderRange(left,right,cursor,boundary);
            const auto& m=metadata.getMessage();
            if(m.isNoteOn())engine.noteOn(m.getNoteNumber(),m.getFloatVelocity());
            else if(m.isNoteOff())engine.noteOff(m.getNoteNumber());
            else if(m.isPitchWheel())engine.pitchWheel(m.getPitchWheelValue());
            else if(m.isAllSoundOff())engine.allSoundOff();
            else if(m.isAllNotesOff()||(m.isController()&&m.getControllerNumber()==123))engine.allNotesOff();
            cursor=boundary;
        }
        renderRange(left,right,cursor,samples);
        midi.clear();
    }
    bool hasEditor()const override{return true;}
    juce::AudioProcessorEditor* createEditor()override;
    void getStateInformation(juce::MemoryBlock& dest)override {
        if(auto xml=parameters.copyState().createXml())copyXmlToBinary(*xml,dest);
    }
    void setStateInformation(const void* data,int size)override {
        if(auto xml=getXmlFromBinary(data,size))
            if(xml->hasTagName(parameters.state.getType()))
                parameters.replaceState(juce::ValueTree::fromXml(*xml));
    }
    Tree parameters;
    juce::MidiKeyboardState keyboardState;
    SpectrumPipe spectrum;
private:
    void renderRange(float* left,float* right,int from,int until) {
        constexpr int FRAME=1024;
        while(from<until) {
            const int n=std::min(FRAME,until-from);
            engine.render(left+from,right+from,n,taps[0].data(),taps[1].data(),taps[2].data());
            spectrum.push(taps[0].data(),taps[1].data(),taps[2].data(),n);
            from+=n;
        }
    }
    float raw(const char* id)const{return parameters.getRawParameterValue(id)->load();}
    void updateEngineParameters() {
        mono64::Parameters p;
        static const char* ids[3][3]={{"osc1_wave","osc1_oct","osc1_level"},
                                    {"osc2_wave","osc2_oct","osc2_level"},
                                    {"osc3_wave","osc3_oct","osc3_level"}};
        for(int i=0;i<3;++i) {
            p.oscillators[i]={static_cast<int>(raw(ids[i][0])),static_cast<int>(raw(ids[i][1])),raw(ids[i][2])};
        }
        p.unison=static_cast<int>(raw("unison"));
        p.detuneCents=raw("detune");p.stereoSpread=raw("spread");
        p.driftDepthCents=raw("drift_depth");p.driftRateHz=raw("drift_rate");
        p.ampVariation=raw("amp_variation");p.glideMs=raw("glide");
        p.pulseWidth=raw("pw")*.01f;
        p.pwmRateHz=raw("pwm_rate");p.pwmDepth=raw("pwm_depth")*.01f;
        p.master=raw("master");p.legato=raw("legato")>=.5f;
        p.subLevel=raw("sub");p.subOctave=raw("sub_oct")>=.5f?-2:-1;
        p.cutoffHz=raw("cutoff");p.resonance=raw("resonance");
        p.filterEnvOctaves=raw("filter_amount");p.drive=raw("drive");
        p.unisonHpOn=raw("hp_full_on")>=.5f;
        p.unisonHpHz=raw("hp_full_cut");p.unisonHp24=raw("hp_full_24")>=.5f;
        p.sideHpOn=raw("hp_side_on")>=.5f;
        p.sideHpHz=raw("hp_side_cut");p.sideHp24=raw("hp_side_24")>=.5f;
        p.ampEnv={raw("amp_attack"),raw("amp_decay"),raw("amp_sustain"),raw("amp_release")};
        p.filterEnv={raw("filter_attack"),raw("filter_decay"),raw("filter_sustain"),raw("filter_release")};
        for(int n=0;n<2;++n) {
            const auto prefix="lfo"+juce::String(n+1)+"_";
            const auto rate=prefix+"rate",depth=prefix+"depth",shape=prefix+"shape",dest=prefix+"dest";
            p.lfos[n]={raw(rate.toRawUTF8()),raw(depth.toRawUTF8())*.01f,
                static_cast<int>(raw(shape.toRawUTF8())),static_cast<int>(raw(dest.toRawUTF8()))};
        }
        engine.setParameters(p);
    }
    std::array<std::array<float,1024>,3> taps{};
    mono64::SynthEngine engine;
};

constexpr juce::uint32 BG = 0xff111318;
constexpr juce::uint32 PANEL = 0xff1d2028;
constexpr juce::uint32 ACCENT = 0xffc7fc66;
constexpr juce::uint32 MUTED = 0xff9199aa;

class IndustrialLook final : public juce::LookAndFeel_V4 {
public:
    IndustrialLook() {
        setColour(juce::Slider::textBoxTextColourId, juce::Colour(0xffe6eaf0));
        setColour(juce::Slider::textBoxOutlineColourId, juce::Colours::transparentBlack);
        setColour(juce::Slider::textBoxBackgroundColourId, juce::Colours::transparentBlack);
        setColour(juce::ComboBox::backgroundColourId, juce::Colour(0xff2c313a));
        setColour(juce::ComboBox::textColourId, juce::Colour(0xffe6eaf0));
        setColour(juce::ComboBox::outlineColourId, juce::Colour(0xff414753));
        setColour(juce::PopupMenu::backgroundColourId, juce::Colour(0xff22262d));
        setColour(juce::PopupMenu::textColourId, juce::Colour(0xfff1f4f8));
        setColour(juce::ToggleButton::textColourId, juce::Colour(0xffd9dfe7));
    }
    void drawRotarySlider(juce::Graphics& g, int x, int y, int width, int height,
                          float proportion, float startAngle, float endAngle,
                          juce::Slider&) override {
        const float cx = x + width * 0.5f;
        const float cy = y + height * 0.5f;
        const float radius = std::min(width, height) * 0.36f;
        const float stroke = std::max(3.0f, radius * 0.115f);
        juce::Path backgroundArc, activeArc;
        backgroundArc.addCentredArc(cx, cy, radius, radius, 0.0f, startAngle, endAngle, true);
        activeArc.addCentredArc(cx, cy, radius, radius, 0.0f, startAngle,
                                startAngle + proportion * (endAngle - startAngle), true);
        g.setColour(juce::Colour(0xff404651));
        g.strokePath(backgroundArc, juce::PathStrokeType(stroke, juce::PathStrokeType::curved,
                                                        juce::PathStrokeType::rounded));
        g.setColour(juce::Colour(ACCENT));
        g.strokePath(activeArc, juce::PathStrokeType(stroke, juce::PathStrokeType::curved,
                                                    juce::PathStrokeType::rounded));
        g.setColour(juce::Colour(0xff2e333d));
        g.fillEllipse(cx - radius * 0.68f, cy - radius * 0.68f, radius * 1.36f, radius * 1.36f);
        float angle = startAngle + proportion * (endAngle - startAngle);
        juce::Point<float> p1(cx + std::sin(angle) * radius * 0.30f, cy - std::cos(angle) * radius * 0.30f);
        juce::Point<float> p2(cx + std::sin(angle) * radius * 0.62f, cy - std::cos(angle) * radius * 0.62f);
        g.setColour(juce::Colours::white);
        g.drawLine(p1.x, p1.y, p2.x, p2.y, 2.4f);
    }
};

class Dial final : public juce::Component {
public:
    Dial(Tree& tree, const char* id, const juce::String& label) : caption(label) {
        slider.setSliderStyle(juce::Slider::RotaryVerticalDrag);
        slider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 100, 22);
        auto* param = tree.getParameter(id);
        slider.setDoubleClickReturnValue(true, param->convertFrom0to1(param->getDefaultValue()));
        addAndMakeVisible(slider);
        attachment = std::make_unique<Tree::SliderAttachment>(tree, id, slider);
    }
    void resized() override { slider.setBounds(1, 20, getWidth()-2, getHeight()-21); }
    void paint(juce::Graphics& g) override {
        g.setColour(juce::Colour(MUTED));
        g.setFont(juce::Font(12.0f, juce::Font::bold));
        g.drawFittedText(caption, getLocalBounds().removeFromTop(22), juce::Justification::centred, 1);
    }
private:
    juce::String caption;
    juce::Slider slider;
    std::unique_ptr<Tree::SliderAttachment> attachment;
};

class Select final : public juce::Component {
public:
    Select(Tree& tree, const char* id, const juce::String& title,
           const juce::StringArray& options) : caption(title) {
        for (int i = 0; i < options.size(); ++i) combo.addItem(options[i], i+1);
        combo.setJustificationType(juce::Justification::centred);
        addAndMakeVisible(combo);
        attachment = std::make_unique<Tree::ComboBoxAttachment>(tree, id, combo);
    }
    void resized() override { combo.setBounds(6, 27, getWidth()-12, 30); }
    void paint(juce::Graphics& g) override {
        g.setColour(juce::Colour(MUTED));
        g.setFont(juce::Font(12.0f, juce::Font::bold));
        g.drawFittedText(caption, getLocalBounds().removeFromTop(23), juce::Justification::centred, 1);
    }
private:
    juce::String caption;
    juce::ComboBox combo;
    std::unique_ptr<Tree::ComboBoxAttachment> attachment;
};

class Switch final : public juce::Component {
public:
    Switch(Tree& tree, const char* id, const juce::String& title) {
        toggle.setButtonText(title);
        addAndMakeVisible(toggle);
        attachment = std::make_unique<Tree::ButtonAttachment>(tree, id, toggle);
    }
    void resized() override { toggle.setBounds(getLocalBounds()); }
private:
    juce::ToggleButton toggle;
    std::unique_ptr<Tree::ButtonAttachment> attachment;
};

class PanelView final : public juce::Component {
public:
    explicit PanelView(juce::String title) : panelTitle(std::move(title)) {}
    void paint(juce::Graphics& g) override {
        auto area = getLocalBounds().toFloat().reduced(1.0f);
        g.setColour(juce::Colour(PANEL));
        g.fillRoundedRectangle(area, 11.0f);
        g.setColour(juce::Colour(0xff343a45));
        g.drawRoundedRectangle(area, 11.0f, 1.0f);
        g.setColour(juce::Colour(ACCENT));
        g.fillRoundedRectangle(18.0f, 17.0f, 4.0f, 17.0f, 2.0f);
        g.setColour(juce::Colour(0xfff3f5f9));
        g.setFont(juce::Font(18.0f, juce::Font::bold));
        g.drawText(panelTitle, 31, 12, getWidth()-50, 27, juce::Justification::left);
    }
    void dial(Tree& tree, const char* id, const char* title, int x, int y, int w=122, int h=125) {
        addControl(std::make_unique<Dial>(tree, id, title), {x, y, w, h});
    }
    void choice(Tree& tree, const char* id, const char* title, const juce::StringArray& options,
                int x, int y, int w=152, int h=64) {
        addControl(std::make_unique<Select>(tree, id, title, options), {x,y,w,h});
    }
    void sw(Tree& tree, const char* id, const char* title, int x, int y, int w=110) {
        addControl(std::make_unique<Switch>(tree, id, title), {x,y,w,26});
    }
    void note(const juce::String& lines,int x,int y,int w,int h) {
        auto label=std::make_unique<juce::Label>();
        label->setText(lines,juce::dontSendNotification);
        label->setJustificationType(juce::Justification::centredLeft);
        label->setColour(juce::Label::textColourId,juce::Colour(MUTED));
        label->setFont(juce::Font(13.f));
        addControl(std::move(label),{x,y,w,h});
    }
private:
    void addControl(std::unique_ptr<juce::Component> component, juce::Rectangle<int> area) {
        component->setBounds(area);
        addAndMakeVisible(*component);
        controls.push_back(std::move(component));
    }
    juce::String panelTitle;
    std::vector<std::unique_ptr<juce::Component>> controls;
};


class SpectrumView final:public juce::Component,private juce::Timer {
public:
    explicit SpectrumView(SpectrumPipe& pipe):source(pipe),fft(12){
        setOpaque(false);
        startTimerHz(30);
    }
    void paint(juce::Graphics& g)override {
        const auto area=getLocalBounds().toFloat().reduced(1);
        g.setColour(juce::Colour(PANEL));g.fillRoundedRectangle(area,11);
        g.setColour(juce::Colour(0xff343a45));g.drawRoundedRectangle(area,11,1);
        g.setColour(juce::Colour(ACCENT));g.fillRoundedRectangle(18,17,4,17,2);
        g.setFont(juce::Font(18.f,juce::Font::bold));g.setColour(juce::Colours::white);
        g.drawText("00  /  SPECTRUM ANALYZER",31,11,350,29,juce::Justification::left);
        g.setFont(juce::Font(13.f));
        g.setColour(juce::Colour(0xff4ee1e0));g.fillEllipse(753,20,10,10);
        g.drawText("UNISON (L+R)",770,12,155,24,juce::Justification::left);
        g.setColour(juce::Colour(0xffffaf66));g.fillEllipse(947,20,10,10);
        g.drawText("SUB · MONO",963,12,140,24,juce::Justification::left);
        const auto graph=juce::Rectangle<float>(66,48,getWidth()-97.f,getHeight()-87.f);
        g.setColour(juce::Colour(0xff111923));g.fillRoundedRectangle(graph,5);
        g.saveState();g.reduceClipRegion(graph.toNearestInt());
        g.setFont(juce::Font(11.f));
        for(int db=0;db>=-80;db-=20){
            const float y=dbY(db,graph);
            g.setColour(juce::Colour(0xff2e3844));g.drawHorizontalLine(int(y),graph.getX(),graph.getRight());
        }
        const int freqs[]={40,100,300,1000,3000,10000,20000};
        const char* names[]={"40","100","300","1k","3k","10k","20k"};
        for(int i=0;i<7;++i){
            const float x=freqX(float(freqs[i]),graph);
            g.setColour(juce::Colour(0xff2e3844));g.drawVerticalLine(int(x),graph.getY(),graph.getBottom());
        }
        if(haveData){
            drawSpectrum(g,graph,unisonBins,juce::Colour(0xff4ee1e0));
            drawSpectrum(g,graph,subBins,juce::Colour(0xffffaf66));
        }
        g.restoreState();
        g.setFont(juce::Font(11.f));g.setColour(juce::Colour(0xff80909e));
        for(int db=0;db>=-80;db-=20){
            const float y=dbY(float(db),graph);
            g.drawText(juce::String(db)+" dB",9,int(y)-8,53,17,juce::Justification::right);
        }
        g.setColour(juce::Colour(0xff97a7b7));
        for(int i=0;i<7;++i){
            float x=freqX(float(freqs[i]),graph);
            g.drawText(names[i],int(x)-20,int(graph.getBottom()+6),40,15,juce::Justification::centred);
        }
        g.setColour(juce::Colour(0xff87919f));g.setFont(juce::Font(12.f));
        g.drawText("FFT 4096  •  30 fps  •  signals measured AFTER both high-pass filters, before mix",
                   66,getHeight()-21,getWidth()-130,17,juce::Justification::centredLeft);
    }
private:
    static constexpr int ORDER=12,FFT_SIZE=1<<ORDER,BINS=FFT_SIZE/2;
    static constexpr int POINTS=300;
    static float dbY(float db,juce::Rectangle<float> r){return r.getY()+(1-juce::jlimit(0.f,1.f,(db+80)/86.f))*r.getHeight();}
    static float freqX(float hz,juce::Rectangle<float> r){
        return r.getX()+r.getWidth()*static_cast<float>(std::log(hz/30.)/std::log(20000./30.));
    }
    void timerCallback()override {
        int got=0;
        while((got=source.pop(batch.data(),static_cast<int>(batch.size())))>0){
            for(int i=0;i<got;++i){
                const auto& f=batch[static_cast<size_t>(i)];
                history[0][position]=f.unisonL;
                history[1][position]=f.unisonR;
                history[2][position]=f.sub;
                position=(position+1)&(FFT_SIZE-1);
                filled=std::min(filled+1,FFT_SIZE);
            }
        }
        if(filled<FFT_SIZE)return;
        const double sr=std::max(1000.,source.getSampleRate());
        for(int ch=0;ch<3;++ch){
            auto& data=fftData[ch];
            for(int n=0;n<FFT_SIZE;++n){
                const int ix=(position+n)&(FFT_SIZE-1);
                const float hann=.5f-.5f*std::cos(2.f*juce::MathConstants<float>::pi*n/(FFT_SIZE-1));
                data[n]=history[ch][ix]*hann;
            }
            std::fill(data.begin()+FFT_SIZE,data.end(),0.f);
            fft.performFrequencyOnlyForwardTransform(data.data());
        }
        for(int i=0;i<POINTS;++i){
            const double f=30.*std::exp(std::log(20000./30.)*i/(POINTS-1.));
            const int ix=juce::jlimit(1,BINS-2,static_cast<int>(std::round(f*FFT_SIZE/sr)));
            double magL=0,magR=0,magSub=0;
            for(int k=ix-1;k<=ix+1;++k){
                magL=std::max(magL,double(fftData[0][k]));
                magR=std::max(magR,double(fftData[1][k]));
                magSub=std::max(magSub,double(fftData[2][k]));
            }
            const double magU=std::sqrt((magL*magL+magR*magR)/2.);
            // Hann coherent-gain compensation; spectrum may show close to 0 dBFS.
            const float dbU=static_cast<float>(20*std::log10(magU*2./FFT_SIZE+1e-6));
            const float dbS=static_cast<float>(20*std::log10(magSub*2./FFT_SIZE+1e-6));
            unisonBins[i]=std::max(dbU,unisonBins[i]-.9f);
            subBins[i]=std::max(dbS,subBins[i]-.9f);
        }
        haveData=true;repaint();
    }
    void drawSpectrum(juce::Graphics& g,juce::Rectangle<float> graph,
                      const std::array<float,POINTS>& levels,juce::Colour color){
        juce::Path path;
        for(int i=0;i<POINTS;++i){
            float x=graph.getX()+graph.getWidth()*i/(POINTS-1.f);
            float y=dbY(levels[static_cast<size_t>(i)],graph);
            if(i==0)path.startNewSubPath(x,y);else path.lineTo(x,y);
        }
        g.setColour(color.withAlpha(.13f));
        auto fill=path;
        fill.lineTo(graph.getRight(),graph.getBottom());
        fill.lineTo(graph.getX(),graph.getBottom());fill.closeSubPath();
        g.fillPath(fill);
        g.setColour(color);g.strokePath(path,juce::PathStrokeType(1.65f));
    }
    SpectrumPipe& source;
    juce::dsp::FFT fft;
    std::array<SpectrumFrame,8192> batch{};
    std::array<std::array<float,FFT_SIZE>,3> history{};
    std::array<std::array<float,FFT_SIZE*2>,3> fftData{};
    std::array<float,POINTS> unisonBins{},subBins{};
    int position=0,filled=0;
    bool haveData=false;
};

class PanelCanvas final:public juce::Component {
public:
    PanelCanvas(Tree& tree,SpectrumPipe& pipe){
        setSize(1148,1665);
        auto spectrum=std::make_unique<SpectrumView>(pipe);
        spectrum->setBounds(15,12,1116,265);
        addAndMakeVisible(*spectrum);
        analyzer=std::move(spectrum);
        auto add=[&](const juce::String& title,int x,int y,int w,int h)->PanelView& {
            auto p=std::make_unique<PanelView>(title);
            p->setBounds(x,y,w,h);addAndMakeVisible(*p);
            panels.push_back(std::move(p));return *panels.back();
        };
        {
            auto& u=add("01  /  64-VOICE UNISON ENGINE",15,289,1116,176);
            const char* ids[]={"unison","detune","spread","drift_depth","drift_rate","amp_variation","glide","master"};
            const char* labels[]={"VOICES","DETUNE","SPREAD","DRIFT DEPTH","DRIFT RATE","AMP VAR","GLIDE","MASTER"};
            for(int i=0;i<8;++i)u.dial(tree,ids[i],labels[i],16+i*136,40,130,124);
            u.sw(tree,"legato","LEGATO",986,11,110);
        }
        const juce::StringArray oscWaves{"Saw","Square"};
        for(int n=1;n<=3;++n){
            auto& p=add("0"+juce::String(n+1)+"  /  OSCILLATOR "+juce::String(n),
                        15+(n-1)*376,477,364,182);
            const auto prefix="osc"+juce::String(n)+"_";
            auto wave=prefix+"wave",oct=prefix+"oct",level=prefix+"level";
            p.choice(tree,wave.toRawUTF8(),"WAVE",oscWaves,8,55,132);
            p.dial(tree,oct.toRawUTF8(),"OCTAVE",141,44,99,126);
            p.dial(tree,level.toRawUTF8(),"LEVEL",241,44,112,126);
        }
        {
            auto& pw=add("05  /  GLOBAL PWM + SAW SHAPE  ·  ALL 3 UNISON OSCILLATORS",15,671,1116,176);
            pw.dial(tree,"pw","PW / SHAPE %",50,40,176,125);
            pw.dial(tree,"pwm_rate","MOD RATE (Hz)",278,40,176,125);
            pw.dial(tree,"pwm_depth","MOD DEPTH %",506,40,176,125);
            pw.note("50% = CLASSIC WAVES\nSQUARE: PULSE WIDTH\nSAW: RAMP SKEW\nLFO 1/2 CAN TARGET SHAPE",728,57,366,97);
        }
        {
            auto& hp=add("06  /  DUAL HIGH-PASS  ·  UNISON ONLY  ·  SUB BYPASSES",15,859,1116,194);
            hp.sw(tree,"hp_full_on","UNISON HP ON",32,62,161);
            hp.dial(tree,"hp_full_cut","UNISON CUT (Hz)",214,48,158,128);
            hp.sw(tree,"hp_full_24","24 dB / OCT",389,62,155);
            hp.sw(tree,"hp_side_on","SIDE HP ON",593,62,145);
            hp.dial(tree,"hp_side_cut","SIDE CUT (Hz)",757,48,158,128);
            hp.sw(tree,"hp_side_24","24 dB / OCT",926,62,150);
        }
        {
            auto& sub=add("07  /  ONE-VOICE SUB · SINE · MONO",15,1065,364,181);
            sub.dial(tree,"sub","LEVEL",20,43,145,127);
            sub.choice(tree,"sub_oct","PITCH",{"-1 octave","-2 octaves"},187,76,160);
        }
        {
            auto& f=add("08  /  UNISON LOW-PASS & DRIVE",391,1065,740,181);
            const char* ids[]={"cutoff","resonance","filter_amount","drive"};
            const char* names[]={"CUTOFF","RESONANCE","ENV AMOUNT","SATURATION"};
            for(int i=0;i<4;++i)f.dial(tree,ids[i],names[i],24+i*177,43,147,126);
        }
        const juce::StringArray lfoWaves{"Sine","Triangle","S&H"};
        const juce::StringArray targets{"Pitch","Filter","Amplitude","PWM / Shape"};
        for(int n=1;n<=2;++n){
            auto& p=add(juce::String(8+n).paddedLeft('0',2)+"  /  LFO "+juce::String(n),
                        15+(n-1)*565,1258,552,194);
            auto prefix="lfo"+juce::String(n)+"_";
            auto rate=prefix+"rate",depth=prefix+"depth",shape=prefix+"shape",dest=prefix+"dest";
            p.dial(tree,rate.toRawUTF8(),"FREQUENCY",14,47,125,129);
            p.dial(tree,depth.toRawUTF8(),"DEPTH",144,47,125,129);
            p.choice(tree,shape.toRawUTF8(),"WAVE",lfoWaves,281,50,120);
            p.choice(tree,dest.toRawUTF8(),"TARGET",targets,409,50,131);
        }
        for(int n=0;n<2;++n){
            auto& p=add(n==0?"11  /  AMPLITUDE ENVELOPE":"12  /  FILTER ENVELOPE",
                        15+n*565,1464,552,183);
            const char* suffix[]={"_attack","_decay","_sustain","_release"};
            const char* labels[]={"ATTACK","DECAY","SUSTAIN","RELEASE"};
            for(int i=0;i<4;++i){
                const auto id=juce::String(n==0?"amp":"filter")+suffix[i];
                p.dial(tree,id.toRawUTF8(),labels[i],17+i*131,46,122,125);
            }
        }
    }
    void paint(juce::Graphics& g)override{g.fillAll(juce::Colour(BG));}
private:
    std::unique_ptr<SpectrumView> analyzer;
    std::vector<std::unique_ptr<PanelView>> panels;
};

class Mono64Editor final:public juce::AudioProcessorEditor {
public:
    explicit Mono64Editor(Mono64Processor& p):juce::AudioProcessorEditor(p),
        content(p.parameters,p.spectrum),
        keyboard(p.keyboardState,juce::MidiKeyboardComponent::horizontalKeyboard){
        setLookAndFeel(&look);
        viewport.setViewedComponent(&content,false);
        viewport.setScrollBarsShown(true,false);
        addAndMakeVisible(viewport);
        keyboard.setAvailableRange(24,96);
        keyboard.setLowestVisibleKey(36);
        keyboard.setKeyWidth(19.f);
        addAndMakeVisible(keyboard);
        setSize(1180,840);
    }
    ~Mono64Editor()override{setLookAndFeel(nullptr);}
    void resized()override{
        viewport.setBounds(15,84,getWidth()-30,getHeight()-184);
        keyboard.setBounds(24,getHeight()-84,getWidth()-48,69);
    }
    void paint(juce::Graphics& g)override{
        g.fillAll(juce::Colour(BG));
        g.setColour(juce::Colour(ACCENT));g.setFont(juce::Font(39.f,juce::Font::bold));
        g.drawText("MONO64",29,10,230,57,juce::Justification::left);
        g.setColour(juce::Colour(MUTED));g.setFont(juce::Font(13.f,juce::Font::bold));
        g.drawText("SAW + SQUARE  /  64 VOICES  /  MONO SUB",265,20,590,25,juce::Justification::left);
        g.drawText("V0.3    ·    GLOBAL PWM/SHAPE    ·    DUAL HP    ·    TWO-COLOR FFT",265,43,730,20,juce::Justification::left);
        g.setColour(juce::Colour(0xff303640));g.drawHorizontalLine(77,25.f,getWidth()-25.f);
        g.setColour(juce::Colour(MUTED));g.setFont(juce::Font(12.f));
        g.drawText("MIDI KEYBOARD  •  PLAY IN FL STUDIO OR CLICK KEYS BELOW",28,
                   getHeight()-110,650,21,juce::Justification::left);
    }
private:
    IndustrialLook look;
    PanelCanvas content;
    juce::Viewport viewport;
    juce::MidiKeyboardComponent keyboard;
};
juce::AudioProcessorEditor* Mono64Processor::createEditor(){return new Mono64Editor(*this);}
} // namespace
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter(){return new Mono64Processor();}
