# /sp.plan - Generate Technical Implementation Plan

## Description
Creates a detailed technical implementation plan for a specified feature. The plan addresses architecture, dependencies, interfaces, non-functional requirements, and risk mitigation strategies.

## Usage
```
/sp.plan <feature-name>
```

## Parameters
- `feature-name` (required): Name of the feature to plan

## Process
1. Reviews feature specification for requirements
2. Designs system architecture and component interactions
3. Identifies technology stack and external dependencies
4. Plans data flow and interface contracts
5. Addresses non-functional requirements (performance, security, reliability)
6. Creates risk analysis and mitigation strategies
7. Establishes evaluation and validation criteria
8. Creates Prompt History Record (PHR) for planning

## Output Files
- `specs/[feature-name]/plan.md` - Technical implementation plan
- `history/prompts/[feature-name]/[ID]-[title].plan.prompt.md` - PHR record

## SDD Compliance Check
- [ ] Architecture decisions documented with rationale
- [ ] Interface contracts clearly defined
- [ ] Non-functional requirements addressed
- [ ] Risk analysis complete with mitigation strategies
- [ ] Evaluation criteria established and testable
- [ ] Alignment with specification verified