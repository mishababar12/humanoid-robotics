import sys
from pathlib import Path
import cohere
from typing import List, Dict, Optional

# Add backend directory to path for imports
backend_dir = Path(__file__).parent.parent
if str(backend_dir) not in sys.path:
    sys.path.insert(0, str(backend_dir))

from database.qdrant import get_qdrant_service
from ingestion.embedder import EmbedderService
from config import settings


class GenerationService:
    """
    Service to generate responses using Cohere API based on retrieved context
    """

    def __init__(self):
        self.co = cohere.Client(settings.cohere_api_key)
        self.model = settings.cohere_generation_model
        self.embedder = EmbedderService()

    def generate_response(self, query: str, search_results: List[Dict], selected_text: Optional[str] = None) -> Dict:
        """
        Generate a response based on query and search results using Cohere Chat API
        """
        try:
            # Build context from search results
            context_text = self._build_context_from_results(search_results)

            # If selected text is provided, prioritize it in the context
            if selected_text:
                full_context = f"Selected Text Context: {selected_text}\n\nRelevant Information: {context_text}"
            else:
                full_context = f"Relevant Information: {context_text}"

            # Create a message that instructs the model to use only provided context
            message = f"""Based on the following information, please answer the question. Only use the information provided below and do not make up any information that is not in the provided context.

INFORMATION CONTEXT:
{full_context}

QUESTION:
{query}

ANSWER:"""

            # Generate response using Cohere Chat API (replaces deprecated Generate API)
            response = self.co.chat(
                message=message,
                model=self.model,
                max_tokens=500,
                temperature=0.3,  # Lower temperature for more factual responses
            )

            if response.text:
                generated_text = response.text.strip()

                # Extract sources from search results
                sources = []
                for result in search_results:
                    source_info = {
                        "title": result.get("title", ""),
                        "url": result.get("url", ""),
                        "relevance_score": result.get("relevance_score", 0.0)
                    }
                    sources.append(source_info)

                return {
                    "response": generated_text,
                    "sources": sources,
                    "context_used": len(search_results)
                }
            else:
                return {
                    "response": "I couldn't generate a response based on the available information.",
                    "sources": [],
                    "context_used": 0
                }

        except Exception as e:
            print(f"Error generating response: {e}")
            return {
                "response": "I encountered an error while generating a response. Please try again.",
                "sources": [],
                "context_used": 0
            }

    def _build_context_from_results(self, results: List[Dict]) -> str:
        """
        Build a context string from search results
        """
        context_parts = []
        for result in results:
            content = result.get("content", "")
            title = result.get("title", "")
            if content:
                context_parts.append(f"Source: {title}\nContent: {content}\n")

        return "\n".join(context_parts)

    def _create_prompt(self, query: str, context: str) -> str:
        """
        Create a prompt that instructs the model to use only provided context
        """
        prompt = f"""Based on the following information, please answer the question. Only use the information provided below and do not make up any information that is not in the provided context.

INFORMATION CONTEXT:
{context}

QUESTION:
{query}

ANSWER:"""

        return prompt

    def generate_response_with_citations(self, query: str, search_results: List[Dict], selected_text: Optional[str] = None) -> Dict:
        """
        Generate response with proper citations to sources
        """
        response_data = self.generate_response(query, search_results, selected_text)

        # Add more detailed source information
        detailed_sources = []
        for result in search_results:
            detailed_source = {
                "title": result.get("title", "Unknown Source"),
                "url": result.get("url", ""),
                "content_preview": result.get("content", "")[:200] + "..." if len(result.get("content", "")) > 200 else result.get("content", ""),
                "relevance_score": result.get("relevance_score", 0.0),
                "source": result.get("source", "")
            }
            detailed_sources.append(detailed_source)

        response_data["detailed_sources"] = detailed_sources
        return response_data


# Global instance
generation_service = GenerationService()