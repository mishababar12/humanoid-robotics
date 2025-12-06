import React from 'react';
import Content from '@theme-original/DocItem/Content';

// This is a conceptual example of how to swizzle the DocItem/Content component
// to dynamically adjust the content based on the user's background.

// In a real application, you would need a more sophisticated way to
// identify and replace content blocks. This could be done with a
// custom markdown syntax or by post-processing the HTML.

const adjustContent = (content, userBackground) => {
    if (!userBackground || typeof content !== 'string') {
        return content;
    }

    // This is a very simplified example. In a real application, you would
    // likely have a more structured way of defining personalized content.
    if (userBackground === 'software') {
        // Hide content intended for hardware experts
        return content.replace(/<div class="hardware-only">.*?<\/div>/gs, '');
    } else if (userBackground === 'hardware') {
        // Hide content intended for software experts
        return content.replace(/<div class="software-only">.*?<\/div>/gs, '');
    }

    return content;
};

export default function ContentWrapper(props) {
    // In a real application, you would get the user's background from your
    // authentication service.
    const userBackground = 'software'; // Dummy data

    const adjustedContent = adjustContent(props.children, userBackground);

    return (
        <Content {...props}>
            {adjustedContent}
        </Content>
    );
}
