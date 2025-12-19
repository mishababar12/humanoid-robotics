import { betterAuth } from "better-auth";

export const auth = betterAuth({
  database: {
    provider: "sqlite",
    url: process.env.DATABASE_URL || "./db.sqlite",
  },
  secret: process.env.AUTH_SECRET || "fallback-secret-change-me",
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },
  socialProviders: {
    // Add social providers if needed
  },
  user: {
    // Additional user fields for background questionnaire
    additionalFields: {
      expertiseLevel: {
        type: "string",
        required: false,
      },
      background: {
        type: "string",
        required: false,
      },
      softwareExperience: {
        type: "string",
        required: false,
      },
      hardwareExperience: {
        type: "string",
        required: false,
      },
    },
  },
});