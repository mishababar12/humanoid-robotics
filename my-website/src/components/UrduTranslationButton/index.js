import React, {useState} from 'react';
import translateText from '../../utils/translation';

// This is a conceptual example of a translation button component.
// In a real application, you would need to get the page content
// and replace it with the translated content.

const UrduTranslationButton = () => {
    const [isTranslated, setIsTranslated] = useState(false);
    const [originalContent, setOriginalContent] = useState('');
    const [translatedContent, setTranslatedContent] = useState('');

    const handleTranslate = async () => {
        if (isTranslated) {
            // Revert to original content
            document.body.innerHTML = originalContent;
            setIsTranslated(false);
        } else {
            // Translate to Urdu
            const pageContent = document.body.innerHTML;
            setOriginalContent(pageContent);
            const translated = await translateText(pageContent, 'ur');
            setTranslatedContent(translated);
            document.body.innerHTML = translated;
            setIsTranslated(true);
        }
    };

    return (
        <button onClick={handleTranslate}>
            {isTranslated ? 'Show Original' : 'Translate to Urdu'}
        </button>
    );
};

export default UrduTranslationButton;
