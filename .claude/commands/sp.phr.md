# /sp.phr - Create Prompt History Record

## Description
Creates a Prompt History Record (PHR) documenting the user input, assistant response, and context for a specific development activity. PHRs are essential for maintaining development traceability and knowledge management.

## Usage
```
/sp.phr <title> --stage <stage> [--feature <feature-name>] [--files <file1,file2>] [--tests <test1,test2>]
```

## Parameters
- `title` (required): Brief descriptive title for the PHR
- `--stage` (required): Development stage (constitution|spec|plan|tasks|red|green|refactor|explainer|misc|general)
- `--feature` (optional): Feature name for routing (default: general)
- `--files` (optional): Comma-separated list of affected files
- `--tests` (optional): Comma-separated list of affected tests

## Process
1. Captures current user prompt verbatim
2. Documents the assistant's approach and decisions
3. Records affected files and changes made
4. Links to related specifications, plans, or tasks
5. Creates properly routed PHR file with metadata

## Output Files
- `history/prompts/[route]/[ID]-[slug].[stage].prompt.md` - PHR record

## SDD Compliance Check
- [ ] User prompt captured verbatim
- [ ] Assistant response documented
- [ ] Files and changes properly recorded
- [ ] Proper routing based on feature/stage
- [ ] Metadata complete and accurate
- [ ] Links to related artifacts included