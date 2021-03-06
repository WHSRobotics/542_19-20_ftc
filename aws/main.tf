provider "aws" {
  region     = "us-east-1"
  profile    = "ftc"
  access_key = var.access_key
  secret_key = var.secret_key
}

terraform {
  backend "s3" {
    bucket = "ftc-542-android-1"
    key    = "terraform/infrastructure.tfstate"
    region = "us-east-1"
  }
}

resource "aws_s3_bucket" "ftc" {
  bucket        = "ftc-542-android-1"
  acl           = "log-delivery-write"
  force_destroy = "false"

  lifecycle_rule {
    abort_incomplete_multipart_upload_days = 0
    enabled                                = true
    id                                     = "Archive-Logs"
    prefix                                 = "logs"
    tags                                   = {}
    expiration {
      days                         = 14
      expired_object_delete_marker = false
    }
  }

  tags = {
    Name = "ftc-542-android-1"
  }
}

resource "aws_iam_role" "codebuild" {
  name               = "codebuild-FTC-19-20-service-role"
  path               = "/service-role/"
  assume_role_policy = <<EOF
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Principal": {
        "Service": "codebuild.amazonaws.com"
      },
      "Action": "sts:AssumeRole"
    }
  ]
}
EOF
}

resource "aws_iam_policy" "codebuild" {
  name   = "codebuild-FTC-19-20"
  policy = <<POLICY
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Effect": "Allow",
            "Resource": [
                "arn:aws:logs:us-east-1:970908632829:log-group:/aws/codebuild/FTC-542-19-20",
                "arn:aws:logs:us-east-1:970908632829:log-group:/aws/codebuild/FTC-542-19-20:*"
            ],
            "Action": [
                "logs:CreateLogGroup",
                "logs:CreateLogStream",
                "logs:PutLogEvents"
            ]
        },
        {
            "Effect": "Allow",
            "Resource": [
                "arn:aws:s3:::codepipeline-us-east-1-*"
            ],
            "Action": [
                "s3:PutObject",
                "s3:GetObject",
                "s3:GetObjectVersion",
                "s3:GetBucketAcl",
                "s3:GetBucketLocation"
            ]
        },
        {
            "Effect": "Allow",
            "Resource": [
                "arn:aws:s3:::ftc-542-android-1",
                "arn:aws:s3:::ftc-542-android-1/*"
            ],
            "Action": [
                "s3:PutObject",
                "s3:GetBucketAcl",
                "s3:GetBucketLocation"
            ]
        }
    ]
}
POLICY
}

resource "aws_iam_role_policy_attachment" "codebuild" {
  role       = "${aws_iam_role.codebuild.name}"
  policy_arn = "${aws_iam_policy.codebuild.arn}"
}

resource "aws_codebuild_project" "ftc-codebuild" {
  name          = "FTC-542-19-20"
  description   = "2019-2020 Season"
  build_timeout = "60"
  service_role  = "arn:aws:iam::970908632829:role/service-role/codebuild-FTC-19-20-service-role"

  artifacts {
    encryption_disabled    = true
    location               = "ftc-542-android-1"
    name                   = "FTC-542-19-20-artifacts"
    namespace_type         = "NONE"
    override_artifact_name = true
    packaging              = "NONE"
    type                   = "S3"
  }

  cache {
    modes = []
    type  = "NO_CACHE"
  }

  environment {
    compute_type                = "BUILD_GENERAL1_SMALL"
    image                       = "thyrlian/android-sdk"
    image_pull_credentials_type = "SERVICE_ROLE"
    privileged_mode             = false
    type                        = "LINUX_CONTAINER"
  }

  logs_config {
    cloudwatch_logs {
      status = "DISABLED"
    }
    s3_logs {
      encryption_disabled = false
      location            = "ftc-542-android-1/logs"
      status              = "ENABLED"
    }
  }

  source {
    git_clone_depth     = 1
    insecure_ssl        = false
    location            = "https://github.com/WHSRobotics/542_19-20_ftc.git"
    report_build_status = true
    type                = "GITHUB"
  }

  depends_on = [aws_iam_role.codebuild]
  tags = {
    Name = "FTC-542-19-20"
  }
}

resource "aws_codebuild_webhook" "ftc" {
  project_name = "FTC-542-19-20"
  depends_on   = [aws_codebuild_project.ftc-codebuild]
  filter_group {
    filter {
      type    = "EVENT"
      pattern = "PUSH"
    }

    filter {
      type    = "HEAD_REF"
      pattern = "master"
    }
  }
}
