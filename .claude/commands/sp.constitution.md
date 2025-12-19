# /sp.constitution - Generate Project Constitution

## Description
Creates or updates the project constitution file following SDD principles. The constitution defines core principles, technical requirements, governance, and serves as the foundational document that supersedes all other project practices.

## Usage
```
/sp.constitution [feature-name]
```

## Parameters
- `feature-name` (optional): Name of the specific feature or module for targeted constitution

## Process
1. Analyzes existing project structure and current constitution
2. Generates updated constitution based on project evolution
3. Creates Prompt History Record (PHR) for constitution update
4. Validates constitution alignment with SDD principles
5. Reviews constitution with user for approval

## Output Files
- `.specify/memory/constitution.md` - Updated project constitution
- `history/prompts/constitution/[ID]-[title].constitution.prompt.md` - PHR record

## SDD Compliance Check
- [ ] Core principles clearly defined and measurable
- [ ] Technical requirements specified with constraints
- [ ] Governance procedures documented
- [ ] Alignment with overall project vision confirmed