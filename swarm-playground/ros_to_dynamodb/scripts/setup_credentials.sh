#!/bin/bash
##############################################################################
# AWS Credentials Setup Helper
# 
# This script helps you set up your AWS credentials
# Usage: source setup_credentials.sh
##############################################################################

echo "========================================================================"
echo "🔐 AWS Credentials Setup"
echo "========================================================================"
echo ""
echo "Paste your AWS credentials below:"
echo ""

# Prompt for credentials
read -p "AWS_ACCESS_KEY_ID: " access_key
read -p "AWS_SECRET_ACCESS_KEY: " secret_key
read -p "AWS_SESSION_TOKEN: " session_token
read -p "AWS_DEFAULT_REGION [us-east-1]: " region

# Set default region if not provided
if [ -z "$region" ]; then
    region="us-east-1"
fi

# Export credentials
export AWS_ACCESS_KEY_ID="$access_key"
export AWS_SECRET_ACCESS_KEY="$secret_key"
export AWS_SESSION_TOKEN="$session_token"
export AWS_DEFAULT_REGION="$region"

echo ""
echo "✓ Credentials set for this session"
echo ""
echo "To verify, run: aws sts get-caller-identity"
echo ""

