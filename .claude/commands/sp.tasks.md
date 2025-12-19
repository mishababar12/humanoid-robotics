# /sp.tasks - Generate Implementation Task List

## Description
Creates a detailed, testable task list for implementing a feature. Each task is designed to be small, verifiable, and contribute to the overall feature implementation following SDD principles.

## Usage
```
/sp.tasks <feature-name>
```

## Parameters
- `feature-name` (required): Name of the feature to break into tasks

## Process
1. Reviews feature specification and technical plan
2. Breaks implementation into small, testable tasks
3. Defines acceptance criteria for each task
4. Establishes task dependencies and execution order
5. Creates test scenarios for each task
6. Creates Prompt History Record (PHR) for tasks

## Output Files
- `specs/[feature-name]/tasks.md` - Detailed task breakdown
- `history/prompts/[feature-name]/[ID]-[title].tasks.prompt.md` - PHR record

## SDD Compliance Check
- [ ] Tasks are small and testable
- [ ] Acceptance criteria defined for each task
- [ ] Dependencies properly identified
- [ ] Test scenarios created for each task
- [ ] Tasks align with specification and plan
- [ ] Overall implementation path clear and achievable