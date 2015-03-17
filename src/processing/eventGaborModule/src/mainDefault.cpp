#include "mainDefault.h"

mainDefault::mainDefault()
{

    neuronLIF neuron1;
    neuron1.setNeuronId("neuron1");
    neuron1.setDebug(false);
    neuron1.save2File(false);
    neuron1.setNeuronCenter(15.0, 15.0);
    neurons.push_back(&neuron1); 

    neuronLIF neuron2;
    neuron2.setNeuronId("neuron2");
    neuron2.setDebug(false);
    neuron2.save2File(false);
    neuron2.setNeuronCenter(15.0, 47.0);
    neurons.push_back(&neuron2); 
    
    neuronLIF neuron3;
    neuron3.setNeuronId("neuron3");
    neuron3.setDebug(false);
    neuron3.save2File(false);
    neuron3.setNeuronCenter(15.0, 79.0);
    neurons.push_back(&neuron3); 
    
    neuronLIF neuron4;
    neuron4.setNeuronId("neuron4");
    neuron4.setDebug(false);
    neuron4.save2File(false);
    neuron4.setNeuronCenter(15.0, 111.0);
    neurons.push_back(&neuron4); 

    numNeurons = neurons.size();
}

mainDefault::~mainDefault()
{
}
