import os
import re
from pathlib import Path
from typing import List, Dict, Optional
from bs4 import BeautifulSoup
import markdown
from config import settings


class ContentExtractor:
    """
    Extract content from Docusaurus documentation files
    """

    def __init__(self, docs_path: str = "my-website/docs"):
        self.docs_path = Path(docs_path)

    def extract_all_content(self) -> List[Dict]:
        """
        Extract all content from the Docusaurus docs directory
        Returns a list of dictionaries with content and metadata
        """
        all_content = []

        # Process markdown files
        for md_file in self.docs_path.rglob("*.md"):
            if md_file.name.startswith("_"):  # Skip sidebar files
                continue
            content = self._extract_from_markdown(md_file)
            if content:
                all_content.extend(content)

        # Process mdx files (if any)
        for mdx_file in self.docs_path.rglob("*.mdx"):
            if mdx_file.name.startswith("_"):
                continue
            content = self._extract_from_markdown(mdx_file)  # MDX can be processed similarly to MD
            if content:
                all_content.extend(content)

        return all_content

    def _extract_from_markdown(self, file_path: Path) -> List[Dict]:
        """
        Extract content from a markdown file
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract title from frontmatter or first heading
            title = self._extract_title(content)

            # Convert markdown to plain text for processing
            html = markdown.markdown(content)
            soup = BeautifulSoup(html, 'html.parser')
            plain_text = soup.get_text()

            # Extract sections based on headings
            sections = self._extract_sections(content, title, file_path)

            return sections
        except Exception as e:
            print(f"Error extracting content from {file_path}: {e}")
            return []

    def _extract_title(self, content: str) -> str:
        """
        Extract title from markdown frontmatter or first heading
        """
        # Try to extract from frontmatter
        frontmatter_match = re.search(r'^---\s*\n(.*?)\n---', content, re.DOTALL)
        if frontmatter_match:
            frontmatter = frontmatter_match.group(1)
            title_match = re.search(r'title:\s*(.*)', frontmatter)
            if title_match:
                return title_match.group(1).strip().strip('"\'')

        # Try to extract from first heading
        heading_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
        if heading_match:
            return heading_match.group(1).strip()

        # Default to filename
        return "Untitled"

    def _extract_sections(self, content: str, title: str, file_path: Path) -> List[Dict]:
        """
        Extract sections from markdown content based on headings
        """
        # Split content by headings
        lines = content.split('\n')
        sections = []
        current_section = []
        current_heading = title

        for line in lines:
            # Check if this line is a heading
            heading_match = re.match(r'^(#{1,6})\s+(.+)$', line)
            if heading_match:
                # Save previous section if it exists
                if current_section and any(current_section):
                    section_content = '\n'.join(current_section).strip()
                    if section_content:
                        # Convert to relative path from docs directory instead of current directory
                        docs_dir = Path(self.docs_path).resolve()
                        absolute_path = file_path.resolve()
                        try:
                            relative_path = absolute_path.relative_to(docs_dir.parent)
                        except ValueError:
                            # If the file is not within the docs parent directory, use the full relative path
                            relative_path = absolute_path.relative_to(Path('.').resolve())
                        sections.append({
                            'title': current_heading,
                            'content': section_content,
                            'source': str(relative_path),
                            'url': self._generate_url(relative_path, current_heading),
                            'metadata': {
                                'file_path': str(file_path),
                                'section_title': current_heading
                            }
                        })

                # Start new section
                current_heading = heading_match.group(2).strip()
                current_section = [line]
            else:
                current_section.append(line)

        # Add the last section
        if current_section and any(current_section):
            section_content = '\n'.join(current_section).strip()
            if section_content:
                # Convert to relative path from docs directory instead of current directory
                docs_dir = Path(self.docs_path).resolve()
                absolute_path = file_path.resolve()
                try:
                    relative_path = absolute_path.relative_to(docs_dir.parent)
                except ValueError:
                    # If the file is not within the docs parent directory, use the full relative path
                    relative_path = absolute_path.relative_to(Path('.').resolve())
                sections.append({
                    'title': current_heading,
                    'content': section_content,
                    'source': str(relative_path),
                    'url': self._generate_url(relative_path, current_heading),
                    'metadata': {
                        'file_path': str(file_path),
                        'section_title': current_heading
                    }
                })

        return sections

    def _generate_url(self, file_path: Path, heading: str) -> str:
        """
        Generate a URL for the content section
        """
        # Convert file path to URL format (remove .md extension, replace _ with -)
        path_parts = list(file_path.parts)
        if path_parts[-1].endswith('.md'):
            path_parts[-1] = path_parts[-1][:-3]  # Remove .md
        elif path_parts[-1].endswith('.mdx'):
            path_parts[-1] = path_parts[-1][:-4]  # Remove .mdx

        # Convert to URL-friendly format
        url_path = '/'.join(path_parts).replace('_', '-').replace(' ', '-').lower()

        # Add heading anchor if it's not the main title
        heading_anchor = self._slugify(heading)
        if heading_anchor.lower() not in url_path.lower():
            return f"/docs/{url_path}#{heading_anchor}"
        else:
            return f"/docs/{url_path}"

    def _slugify(self, text: str) -> str:
        """
        Convert text to URL-friendly slug
        """
        text = text.lower()
        text = re.sub(r'[^a-z0-9\s-]', '', text)
        text = re.sub(r'\s+', '-', text)
        return text.strip('-')


# Example usage
if __name__ == "__main__":
    extractor = ContentExtractor()
    content = extractor.extract_all_content()
    print(f"Extracted {len(content)} content sections")