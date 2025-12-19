# /sp.specify - Generate Feature Specification

## Description
Creates a detailed specification document for a feature following SDD principles. The specification defines what the feature should do, its requirements, constraints, and acceptance criteria.

## Usage
```
/sp.specify <feature-name>
```

## Parameters
- `feature-name` (required): Name of the feature to specify

## Process
1. Reads existing constitution for context
2. Gathers feature requirements and constraints
3. Defines functional and non-functional requirements
4. Creates acceptance criteria and test scenarios
5. Establishes success metrics and failure conditions
6. Creates Prompt History Record (PHR) for specification

## Output Files
- `specs/[feature-name]/spec.md` - Feature specification document
- `history/prompts/[feature-name]/[ID]-[title].spec.prompt.md` - PHR record

## SDD Compliance Check
- [ ] Requirements clearly defined and testable
- [ ] Constraints and limitations documented
- [ ] Acceptance criteria specific and measurable
- [ ] Alignment with constitution verified
- [ ] Dependencies identified and managed