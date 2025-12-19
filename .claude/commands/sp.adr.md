# /sp.adr - Create Architecture Decision Record

## Description
Creates an Architecture Decision Record (ADR) to document significant architectural decisions, alternatives considered, and rationale. ADRs provide traceability for important technical choices.

## Usage
```
/sp.adr <decision-title> [--status <status>] [--deciders <names>] [--tags <tag1,tag2>]
```

## Parameters
- `decision-title` (required): Clear title for the architectural decision
- `--status` (optional): Status of decision (proposed|accepted|deprecated|superseded, default: accepted)
- `--deciders` (optional): List of decision makers
- `--tags` (optional): Comma-separated tags for categorization

## Process
1. Identifies the architectural decision requiring documentation
2. Documents the context and problem being solved
3. Lists alternatives considered with trade-offs
4. Records the chosen solution and rationale
5. Identifies consequences of the decision
6. Creates proper ADR file with standard template

## Output Files
- `history/adr/[ID]-[slug].md` - Architecture Decision Record

## SDD Compliance Check
- [ ] Context clearly explained
- [ ] Problem statement specific and clear
- [ ] Alternatives considered with trade-offs
- [ ] Chosen solution well-justified
- [ ] Consequences identified
- [ ] Decision status properly tracked