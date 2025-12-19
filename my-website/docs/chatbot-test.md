---
title: RAG Chatbot Test
---

# RAG Chatbot Test Page

This page is designed to test the RAG chatbot functionality. The chatbot should appear as a floating widget on the bottom right of the screen.

You can select any text on this page and ask questions about it using the chatbot. The chatbot will use the selected text as context to provide more accurate answers.

## Example Text for Selection

Here's some sample text that you can select to test the selected text functionality:

> Physical AI represents a paradigm shift from traditional AI systems that operate in virtual environments to AI systems that must interact with the physical world. This requires a fundamentally different approach to intelligence, where cognition emerges from the interaction between an agent and its environment.

Try selecting the above text and asking the chatbot a question about Physical AI.

## How It Works

The RAG (Retrieval-Augmented Generation) chatbot works by:

1. **Retrieving** relevant information from the textbook content
2. **Augmenting** the AI's response with specific citations and sources
3. **Generating** contextually appropriate answers based on the retrieved information

import BrowserOnly from '@docusaurus/BrowserOnly';

<BrowserOnly>
  {() => <RagChatbot />}
</BrowserOnly>

## More Test Content

Here's additional content to test the chatbot with:

Robotics and AI integration requires understanding multiple domains:

- **Perception**: How robots sense and interpret their environment
- **Planning**: How robots decide what actions to take
- **Control**: How robots execute actions in the physical world
- **Learning**: How robots improve their performance over time

Each of these domains presents unique challenges when working with physical systems rather than virtual environments.