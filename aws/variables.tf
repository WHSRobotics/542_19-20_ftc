variable "access_key" {
  type    = string
  default = ""
}

variable "secret_key" {
  type    = string
  default = ""
}

variable "ssh_keyname" {
  default = "aws_mbt" #key you associate with the ec2 instances
}

variable "private_key_location" {
  type = string
  # Add to .bash_profile
  # export TF_VAR_private_key_location="/Users/hudsd020/.ssh/aws_mbt.pem"
}

variable "iam_instance_profile_name" {
  default = "ec2-rostr-qa"
}

