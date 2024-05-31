---
title: Machine Vision in Resource-Constrained Embedded Systems
author:
  - name: Timothy J Wiegman
    institution: Purdue University
    department: Agricultural and Biological Engineering
    orcid: https://orcid.org/0000-0002-4341-0857
    city: West Lafayette, IN
    country: USA
    email: wiegman@purdue.edu
acm:
  - copyrightyear: 2024
    copyright: rightsretained
    conference: Purdue ECE568
    date: Spring 2024
    location: West Lafayette, IN, USA
anonymous: false
date: "2024-04-22"
bibliography: [ece568-hw3.bib]
link-citations: true
secnumdepth: 2
figPrefix: "figure"
secPrefix: "section"
abstract: >-
    Modern machine learning now enables relatively advanced computer vision technology to be deployed on simple, low-resource computing hardware. This allows proliferation of an important type of artificial intelligence to many applications where cost or distribution would have previously been prohibitive. Three recent papers are discussed, showing how such technology is used and developed for embedded platforms. Though this is exciting, it is expected that even more progress is forthcoming both as machine vision software further matures and as advances in computer technology bring more powerful computation to wider distribution and lower cost.
---

# Introduction
Computer vision is an important application of artificial intelligence, a critical technology of the information age. The possible uses for this technology are extremely numerous, from automation to zoology and everything in between [@Szeliski.ComputerVisionAlgorithms.2022; @Smith.QuietRevolutionMachine.2021; @Manoharan.EmbeddedImagingSystem.2020]. However, many of the most popular implementations require powerful computers or other (expensive) specialized equipment, as they are focused on doing the most challenging tasks with the most cutting-edge methods. This means those are unsuited to distributed deployments, real-time requirements, or otherwise scaling beyond niche usage [@Bayoudh.SurveyDeepMultimodal.2022].

However, with the advent of modern machine learning methods, some simplified computer vision models can be run on inexpensive, widespread electronic devices using little more than basic microcontrollers [@Immonen.TinyMachineLearning.2022]. Deploying computer vision technology in this fashion makes it more accessible to the general public, and it is more secure, private, and reliable than outsourcing computation to centralized cloud servers.

# Review
## Prescreening Oral Cancers with TensorFlow Lite For Microcontrollers
The first paper to review in detail is @Shamim.HardwareDeployableEdge.2022. This paper describes a medical application for TensorFlow Lite For Microcontrollers, a "TinyML" [@Immonen.TinyMachineLearning.2022] software that compresses traditional AI models to run on extremely basic ARM devices. The authors re-trained an existing model, originally designed for generic computer vision tasks on mobile devices, for their domain-specific goal of detecting and classifying tongue lesions that can develop into oral cancers. Once fine-tuned in this way, the model was quantized and compressed from 32-bit float to 8-bit integer precision. This converted model, despite being significantly simpler and requiring far less resources (63% lower peak memory and 80% smaller executable), still achieved nearly equivalent accuracy (within 1-2%) and latency (within 0.01 ms). All computation was performed on an ST Microelectronics STM32H743II, a relatively inexpensive and efficient 32-bit microprocessor based on ARM Cortex-M7.

## Embedding Crosswalk Detection into a Wearable Device
The second paper to review in detail is @Silva.MethodEmbeddingComputer.2020. This paper describes the development of a wearable crosswalk detection device, optimized for minimal memory and power consumption. The first portion of the study shows several experiments used to quantify how different choices in the computer vision pipeline---such as the input resolution, color depth, and even the architecture of the machine learning algorithm---affect the memory footprint of the application. The latter portion of the study explained how the application utilized the resources on their embedded platform: the Texas Instruments TM4C123GH6PM, a very inexpensive and efficient ARM Cortex-M4 based microcontroller. The system consumed far less than a watt, and required barely two dozen kilobytes of RAM and flash, and was still able to achieve nearly 90% accuracy with respectable sub-second latency.

## Automating Design of Small-Yet-Powerful Neural Networks
The third and final paper to review in detail is @Liberis.uNasConstrainedNeural.2021. This paper describes a software that can automatically design the architecture for a neural network with the goal of fitting within extremely tight resource constraints, such as those of low-power microcontrollers with slow processors and very little memory available. This is a technology that can enable other engineers to more easily develop AI-powered applications for inexpensive and distributed devices such as Internet-of-Things (IoT) or wearable devices. The authors report that their system can improve model accuracies by a few percent, reduce memory usage multiple-fold, or reduce the number of operations (a proxy for latency) by up to two-fold.

## Comparing the Papers
The first two papers are the most similar. Both describe the development of a prototype embedded device to solve a specific problem. The first was slightly simpler, as it used a higher level computer vision system and merely compressed it for use on a low-power embedded computer. However, the second paper targeted a significantly lower-power device, so the authors of that study explained a much more in-depth process for carefully optimizing their application for their extreme resource constraints. The third and final paper was somewhat abstracted from the others, as it focused on explaining a technology one step higher in the stack. None of the three papers were directly related or cited each other.

# Discussion and Conclusion
I think the types of technologies developed in this field are excellent examples of practical artificial intelligence. While high-power applications like ChatGPT are glossy and visible and exciting, they are unlikely to permeate everyday life like embedded systems do. The ability to integrate cutting-edge computer technologies into low-cost, simple devices will allow them to spread further and faster than expensive and powerful ones. They can solve practical problems, like the second paper demonstrated (in that case, it could be an assistive technology for the visually impaired), while there is also the possibility that they will solve problems we cannot even yet foresee.

The biggest challenges to further rollout of these kinds of technology are likely going to be the difficulty of developing for such tightly resource-constrained hardware, as shown in the in-depth optimization process explained in @Silva.MethodEmbeddingComputer.2020. However, those roadblocks will be eased as better toolchains are built (as in @Liberis.uNasConstrainedNeural.2021) and as powerful computers become cheaper and more widespread purely due to hardware development.

# References
