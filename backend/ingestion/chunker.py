import re
from typing import List, Dict
from config import settings


class ContentChunker:
    """
    Chunk content into semantic boundaries for embedding
    """

    def __init__(self, chunk_size: int = None, chunk_overlap: int = None):
        self.chunk_size = chunk_size or settings.chunk_size
        self.chunk_overlap = chunk_overlap or settings.chunk_overlap

    def chunk_content(self, content_items: List[Dict]) -> List[Dict]:
        """
        Chunk a list of content items into smaller pieces
        Each content item should have: title, content, source, url, metadata
        """
        all_chunks = []

        for item in content_items:
            chunks = self._chunk_single_item(item)
            all_chunks.extend(chunks)

        return all_chunks

    def _chunk_single_item(self, item: Dict) -> List[Dict]:
        """
        Chunk a single content item into smaller pieces
        """
        content = item['content']
        title = item['title']
        source = item['source']
        url = item['url']
        metadata = item.get('metadata', {})

        # Split content into sentences first
        sentences = self._split_into_sentences(content)

        chunks = []
        current_chunk = ""
        current_length = 0

        import uuid
        for sentence in sentences:
            sentence_length = len(sentence)

            # If adding this sentence would exceed chunk size
            if current_length + sentence_length > self.chunk_size and current_chunk:
                # Save current chunk
                chunk_id = str(uuid.uuid4())
                chunks.append({
                    'id': chunk_id,
                    'content': current_chunk.strip(),
                    'title': title,
                    'source': source,
                    'url': url,
                    'metadata': metadata
                })

                # Start new chunk with overlap
                if len(sentences) > 0:
                    # Add overlap from the end of the previous chunk
                    overlap_start = max(0, len(current_chunk) - self.chunk_overlap)
                    current_chunk = current_chunk[overlap_start:] + " " + sentence
                    current_length = len(current_chunk)
                else:
                    current_chunk = sentence
                    current_length = sentence_length
            else:
                # Add sentence to current chunk
                if current_chunk:
                    current_chunk += " " + sentence
                else:
                    current_chunk = sentence
                current_length += sentence_length + 1  # +1 for space

        # Add the final chunk if it has content
        if current_chunk.strip():
            chunk_id = str(uuid.uuid4())
            chunks.append({
                'id': chunk_id,
                'content': current_chunk.strip(),
                'title': title,
                'source': source,
                'url': url,
                'metadata': metadata
            })

        return chunks

    def _split_into_sentences(self, text: str) -> List[str]:
        """
        Split text into sentences while preserving semantic boundaries
        """
        # This is a simple sentence splitter - in production, you might want to use
        # a more sophisticated NLP library like spaCy or NLTK
        sentences = re.split(r'(?<=[.!?])\s+', text)

        # Clean up sentences
        cleaned_sentences = []
        for sentence in sentences:
            sentence = sentence.strip()
            if sentence:
                cleaned_sentences.append(sentence)

        return cleaned_sentences

    def chunk_by_paragraph(self, content_items: List[Dict]) -> List[Dict]:
        """
        Alternative chunking method that splits by paragraphs
        """
        import uuid
        all_chunks = []

        for item in content_items:
            paragraphs = item['content'].split('\n\n')  # Split by double newlines (paragraphs)

            for i, paragraph in enumerate(paragraphs):
                paragraph = paragraph.strip()
                if len(paragraph) > 0:  # Only include non-empty paragraphs
                    chunk_id = str(uuid.uuid4())
                    all_chunks.append({
                        'id': chunk_id,
                        'content': paragraph,
                        'title': item['title'],
                        'source': item['source'],
                        'url': item['url'],
                        'metadata': {**item.get('metadata', {}), 'paragraph_index': i}
                    })

        return all_chunks


# Example usage
if __name__ == "__main__":
    # Example content item
    sample_content = [{
        'title': 'Sample Chapter',
        'content': 'This is the first sentence. This is the second sentence. Here is the third sentence! And this is the fourth sentence? Finally, the fifth sentence.',
        'source': 'sample.md',
        'url': '/docs/sample',
        'metadata': {'file_path': 'sample.md'}
    }]

    chunker = ContentChunker(chunk_size=100, chunk_overlap=20)
    chunks = chunker.chunk_content(sample_content)

    for i, chunk in enumerate(chunks):
        print(f"Chunk {i+1}: {len(chunk['content'])} chars")
        print(f"Content: {chunk['content'][:50]}...")
        print("---")